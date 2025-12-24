from agents import Agent, Runner, OpenAIChatCompletionsModel, AsyncOpenAI
from agents import set_tracing_disabled, function_tool
import os
from dotenv import load_dotenv
from agents import enable_verbose_stdout_logging

# enable_verbose_stdout_logging()  # Uncomment for debugging

load_dotenv()
set_tracing_disabled(disabled=True)

gemini_api_key = os.getenv("GEMINI_API_KEY")
if not gemini_api_key:
    raise ValueError("GEMINI_API_KEY environment variable is required")

provider = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

model = OpenAIChatCompletionsModel(
    model="gemini-2.0-flash-exp",  # Updated to available model
    openai_client=provider
)

import cohere
from qdrant_client import QdrantClient

# Initialize Cohere client with environment variable
cohere_api_key = os.getenv("COHERE_API_KEY")
if not cohere_api_key:
    raise ValueError("COHERE_API_KEY environment variable is required")

cohere_client = cohere.Client(cohere_api_key)

# Connect to Qdrant with environment variables
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
if not qdrant_url or not qdrant_api_key:
    raise ValueError("QDRANT_URL and QDRANT_API_KEY environment variables are required")

qdrant = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key
)

def get_embedding(text):
    """Get embedding vector from Cohere Embed v3"""
    response = cohere_client.embed(
        model="embed-english-v3.0",
        input_type="search_query",  # Use search_query for queries
        texts=[text],
    )
    return response.embeddings[0]  # Return the first embedding

@function_tool
def retrieve(query):
    """Retrieve relevant document chunks from the vector database"""
    try:
        embedding = get_embedding(query)
        result = qdrant.query_points(
            collection_name="naeem",  # Updated to match your .env
            query=embedding,
            limit=5
        )
        chunks = [point.payload.get("content", "") for point in result.points if point.payload]
        if not chunks:
            return ["No relevant documents found in the database. The vector database appears to be empty or the documents haven't been indexed yet."]
        return chunks
    except Exception as e:
        print(f"Error retrieving documents: {e}")
        return [f"Error retrieving documents: {str(e)}"]

agent = Agent(
    name="PhysicalAIAssistant",
    instructions="""
You are an AI tutor for the Physical AI & Humanoid Robotics textbook.
To answer the user question, first call the tool `retrieve` with the user query.
Use ONLY the returned content from `retrieve` to answer.
If the answer is not in the retrieved content, say "I don't know".
Keep answers concise and focused on the textbook content.
""",
    model=model,
    tools=[retrieve]
)

if __name__ == "__main__":
    # Test the agent
    result = Runner.run_sync(
        agent,
        input="what is physical ai?",
    )

    print("Agent Response:")
    print(result.final_output)