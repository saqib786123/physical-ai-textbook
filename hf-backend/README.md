# Physical AI Backend (Hugging Face Edition)

This folder contains the standalone AI backend for the Physical AI & Humanoid Robotics textbook.

## Deployment on Hugging Face Spaces

1. **Create Space**: Choose **Docker** as the SDK.
2. **Configure**: Point the Dockerfile path to `./hf-backend/Dockerfile` in your main repo.
3. **Secrets**: Add `OPENAI_API_KEY` and `JWT_SECRET` in Space Settings.

## Files included:
- `server.js`: The main Express server with RAG and Auth endpoints.
- `Dockerfile`: Container configuration for Hugging Face.
- `package.json`: Dependencies.
- `vector-store/`: Folder for AI knowledge (needs to be populated).
- `users.json`: Local database for prototype users.

## Local Test
```bash
cd hf-backend
npm install
node server.js
```
