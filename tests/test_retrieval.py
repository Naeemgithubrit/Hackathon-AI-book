import cohere
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize Cohere client
cohere_api_key = os.getenv("COHERE_API_KEY")
if not cohere_api_key:
    print("COHERE_API_KEY not found in environment variables")
    exit(1)

cohere_client = cohere.Client(cohere_api_key)

# Connect to Qdrant
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
if not qdrant_url or not qdrant_api_key:
    print("QDRANT_URL or QDRANT_API_KEY not found in environment variables")
    exit(1)

qdrant = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key
)

def get_embedding(text):
    """Get embedding vector from Cohere Embed v3"""
    try:
        response = cohere_client.embed(
            model="embed-english-v3.0",
            input_type="search_query",  # Use search_query for queries
            texts=[text],
        )
        return response.embeddings[0]  # Return the first embedding
    except Exception as e:
        print(f"Error getting embedding: {e}")
        return None

def retrieve(query):
    """Retrieve documents from Qdrant"""
    try:
        embedding = get_embedding(query)
        if embedding is None:
            return ["Error: Could not get embedding"]

        result = qdrant.query_points(
            collection_name="physical-robotics-ai-book",  # Use the correct collection name
            query=embedding,
            limit=5
        )
        return [point.payload.get("content", "No content") for point in result.points]
    except Exception as e:
        return [f"Error retrieving documents: {str(e)}"]

# Test
print("Testing Qdrant retrieval...")
results = retrieve("What data do you have?")
print(f"Found {len(results)} results:")
for i, result in enumerate(results, 1):
    print(f"{i}. {result[:200]}..." if len(result) > 200 else f"{i}. {result}")

# Also check collection info
try:
    collections = qdrant.get_collections()
    print(f"\nAvailable collections: {[c.name for c in collections.collections]}")

    if "physical-robotics-ai-book" in [c.name for c in collections.collections]:
        info = qdrant.get_collection("physical-robotics-ai-book")
        print(f"Collection 'physical-robotics-ai-book' has {info.points_count} points")
    else:
        print("Collection 'physical-robotics-ai-book' not found")
except Exception as e:
    print(f"Error checking collections: {e}")