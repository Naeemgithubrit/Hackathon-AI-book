from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()
qdrant = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
collections = qdrant.get_collections()
print('Collections:', [c.name for c in collections.collections])

if 'physical-robotics-ai-book' in [c.name for c in collections.collections]:
    info = qdrant.get_collection('physical-robotics-ai-book')
    print(f'Collection info: {info.points_count} points')