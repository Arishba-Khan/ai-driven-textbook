from qdrant_client import QdrantClient
from qdrant_client.http import models

c = QdrantClient(":memory:")
c.create_collection("test", vectors_config=models.VectorParams(size=4, distance=models.Distance.COSINE))
c.upsert("test", points=[models.PointStruct(id=1, vector=[0.1, 0.1, 0.1, 0.1], payload={"a": 1})])

res = c.query_points("test", query=[0.1, 0.1, 0.1, 0.1], limit=1, with_payload=True)
print(f"Result type: {type(res)}")
print(f"Result dir: {dir(res)}")
print(f"Result content: {res}")
