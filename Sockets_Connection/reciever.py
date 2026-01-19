import socket
import os
from rdflib import Graph, RDF, RDFS, OWL, URIRef, BNode
import networkx as nx


HOST = "127.0.0.1"
PORT = 9999

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OWL_FILE = os.path.join(BASE_DIR, "fullaxiom.owl")

CMS_NS = "http://example.org/cms#"


print("Loading CORA ontology...")

g = Graph()
g.parse(OWL_FILE)

print("OWL file loaded")
print("Total triples:", len(g))


class_graph = nx.DiGraph()

classes = set()

for c in g.subjects(RDF.type, OWL.Class):
    classes.add(c)

for s, _, _ in g.triples((None, RDFS.subClassOf, None)):
    classes.add(s)

for c in classes:
    class_graph.add_node(str(c))

for s, _, o in g.triples((None, RDFS.subClassOf, None)):
    s_uri = str(s)

    if isinstance(o, URIRef):
        class_graph.add_edge(s_uri, str(o), label="subClassOf")
    elif isinstance(o, BNode):
        class_graph.add_edge(s_uri, str(o), label="restriction")

print("Class graph nodes:", class_graph.number_of_nodes())
print("Class graph edges:", class_graph.number_of_edges())


print("\n--- GRAPH PREVIEW (first 25 edges) ---")
for i, (u, v, data) in enumerate(class_graph.edges(data=True)):
    print(f"{u.split('#')[-1]} --[{data['label']}]--> {v.split('#')[-1]}")
    if i >= 24:
        break


def find_class(name):
    for node in class_graph.nodes:
        if node.endswith("#" + name):
            return node
    return None

def has_path(child, parent):
    try:
        return nx.has_path(class_graph, child, parent)
    except:
        return False


cms_classes = {}

def ensure_cms_class(child, parent):
    """
    Ensure CMS class exists and is linked to CORA parent
    """
    if child in cms_classes:
        return cms_classes[child]

    parent_uri = find_class(parent)
    if not parent_uri:
        return None

    child_uri = CMS_NS + child
    class_graph.add_node(child_uri)
    class_graph.add_edge(child_uri, parent_uri, label="subClassOf")

    cms_classes[child] = child_uri
    return child_uri


server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)

print("\nReceiver started, waiting for Webots controller...")
conn, addr = server.accept()
print("Connected from:", addr)


while True:
    data = conn.recv(1024)
    if not data:
        break

    msg = data.decode().strip()
    print("\n[RAW INPUT]", msg)

    if ":" not in msg:
        continue

    role, symbol = msg.split(":", 1)
    role = role.strip()
    symbol = symbol.strip()

    child_uri = ensure_cms_class(symbol, role)

    if child_uri:
        parent_uri = find_class(role)
        result = has_path(child_uri, parent_uri)

        print(
            f"[ONTOLOGY CHECK] {symbol} ⊑ {role} →",
            "TRUE (present)" if result else "FALSE"
        )
    else:
        print(f"[ONTOLOGY CHECK] FAILED to map {symbol} → {role}")

conn.close()
