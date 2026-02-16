# Chatbot Integration

A simple RAG-like chatbot component has been integrated into the website.

## Features
- Global visibility via `src/theme/Root`.
- Hardcoded knowledge base for immediate use.
- Simple keyword matching logic.
- UI with Open/Close toggle.

## Making it Smarter (True RAG)

To connect this to a real backend (OpenAI, Vector DB, etc.):

1.  **Backend API**: Create an API endpoint (e.g., Vercel Function or AWS Lambda) that accepts a user query.
2.  **Indexing**: Use a script to index all `.md` files in `docs/` and store embeddings in Pinecone/Weaviate.
3.  **Frontend Update**: Modify `src/components/Chatbot/index.tsx`:
    - Replace the `setTimeout` simulation in `handleSend` with a `fetch()` call to your API.
    - Remove the hardcoded `KNOWLEDGE_BASE`.

## Extending Knowledge Base

Edit `src/components/Chatbot/index.tsx` and add more key-value pairs to `KNOWLEDGE_BASE`.

```typescript
const KNOWLEDGE_BASE = {
  ros: "...",
  new_topic: "Explanation...",
  // Add more here
};
```
