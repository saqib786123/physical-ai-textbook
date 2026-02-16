# Chatbot Integration

A RAG-ready chatbot component has been integrated into the website.

## Architecture
- **Frontend**: Docusaurus component in `src/components/Chatbot`.
- **Backend**: Standalone Express server in `backend/server.js`.
- **Knowledge Base**: Generated from `docs/` using `scripts/index-docs.js`.

## How to Run

1.  **Set Environment Variables**:
    Edit `.env.local` and add your `OPENAI_API_KEY`.

2.  **Generate Knowledge Index**:
    ```bash
    node scripts/index-docs.js
    ```
    This creates the vector store in `static/vector-store`.

3.  **Start the Backend Server**:
    ```bash
    cd backend
    node server.js
    ```
    The server runs at `http://localhost:3001`.

4.  **Start Docusaurus**:
    ```bash
    npm start
    ```

## Features
- Global visibility via `src/theme/Root`.
- Automatic fallback to hardcoded knowledge if the backend is down.
- Markdown support in chat messages.

## Making it Smarter
To improve the RAG performance, you can adjust the `chunkSize` and `chunkOverlap` in `scripts/index-docs.js`.
