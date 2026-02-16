# Deployment Guide: Physical AI Textbook (Netlify & Hugging Face)

Follow these steps to deploy your finalized bilingual textbook and AI backend.

## 1. Push Latest Changes to GitHub
```bash
git add .
git commit -m "Add Dockerfile for Hugging Face deployment"
git push origin main
```

## 2. Deploy RAG Backend (Hugging Face Spaces)
1. Sign in to [Hugging Face](https://huggingface.co).
2. Click **New** > **Space**.
3. Name your Space (e.g., `physical-ai-backend`).
4. Select **Docker** as the Space SDK.
5. Choose **Blank** template (or just create the space).
6. Connect your GitHub repository.
7. **Settings**:
   - Ensure the **Dockerfile path** is `./backend/Dockerfile`.
8. **Variables & Secrets**:
   - Go to **Settings** > **Variables and secrets**.
   - **Secret**: `OPENAI_API_KEY` (Your OpenAI key).
   - **Secret**: `JWT_SECRET` (Your random string).
9. Hugging Face will build the Docker image and start the server on port 7860.
10. Your URL will be: `https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space` (e.g., `https://saqib786123-physical-ai-backend.hf.space`).

## 3. Deploy Frontend (Netlify)
1. Sign in to [Netlify.com](https://netlify.com).
2. Connect to GitHub and select your repository.
3. **Environment Variables**:
   - `NEXT_PUBLIC_API_URL`: Paste your **Hugging Face Space URL** here.
4. Click **Deploy site**.

## 4. Final Verification
- Visit your Netlify URL.
- Try the Chatbot and Auth system.

**Note**: Hugging Face Spaces can "sleep" if not used for a while. The first request after a long time might take a few seconds to wake up the server.
