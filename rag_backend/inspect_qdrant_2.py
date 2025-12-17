from qdrant_client import QdrantClient
import inspect

c = QdrantClient(":memory:")
print(f"Has search: {hasattr(c, 'search')}")
print(f"Has query_points: {hasattr(c, 'query_points')}")
try:
    if hasattr(c, 'query_points'):
        print(f"query_points sig: {inspect.signature(c.query_points)}")
    if hasattr(c, 'search'):
        print(f"search sig: {inspect.signature(c.search)}")
except Exception as e:
    print(f"Inspect error: {e}")

print(f"Full dir: {dir(c)}")
