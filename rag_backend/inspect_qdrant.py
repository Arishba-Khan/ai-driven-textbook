import qdrant_client
try:
    print(f"Version: {qdrant_client.__version__}")
except:
    print("Version not found")
    
from qdrant_client import QdrantClient
print(f"QdrantClient detected: {QdrantClient}")
print(f"Attributes: {dir(QdrantClient)}")

try:
    c = QdrantClient(":memory:")
    print(f"Instance attrs: {dir(c)}")
    print(f"Has search: {hasattr(c, 'search')}")
except Exception as e:
    print(f"Error instantiating: {e}")
