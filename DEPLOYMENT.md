# Deployment Guide: Physical AI Textbook (Netlify & Railway)

Follow these steps to deploy your finalized bilingual textbook and AI backend using Netlify and Railway.

## 1. Push Latest Changes to GitHub
Ensure all your latest configuration changes are pushed:
```bash
git add .
git commit -m "Update deployment config for Netlify and Railway"
git push origin main
```

## 2. Deploy RAG Backend (Railway)
1. Sign in to [Railway.app](https://railway.app).
2. Click **New Project** > **Deploy from GitHub repo**.
3. Select your `physical-ai-textbook` repository.
4. Railway will detect the folders. Click on **Settings** for the service.
5. **Root Directory**: Set this to `backend`.
6. **Environment Variables**:
   - `OPENAI_API_KEY`: Your OpenAI Secret Key.
   - `JWT_SECRET`: A long random string for auth.
   - `PORT`: `3001` (Or let Railway assign one, but 3001 is set in our code).
7. Railway will provide a public URL (e.g., `https://backend-production-xxxx.up.railway.app`). Copy this URL.

## 3. Deploy Frontend (Netlify)
1. Sign in to [Netlify.com](https://netlify.com).
2. Click **Add new site** > **Import an existing project**.
3. Connect to GitHub and select the `physical-ai-textbook` repository.
4. **Site Settings**:
   - **Base directory**: (Leave empty, use root)
   - **Build command**: `npm run build`
   - **Publish directory**: `build`
5. **Environment Variables**:
   - `NEXT_PUBLIC_API_URL`: Paste your **Railway Backend URL** here.
6. Click **Deploy site**.

## 4. Final Verification
- Visit your Netlify URL (e.g., `https://physical-ai-textbook.netlify.app`).
- Try to Sign Up / Sign In.
- Ask the Chatbot a question in English.
- Switch to Urdu and ask a question in Urdu.

**Congratulations! Your Physical AI platform is live on the modern stack.**
