# 🚀 Simple & Best Deployment Guide

Follow these steps to deploy your Physical AI Textbook. This setup uses **Netlify** for the frontend and **Render** for the AI backend.

## 1. Push Code to GitHub
Ensure your latest work is on GitHub:
```bash
git add .
git commit -m "Simplify project structure and optimize for production"
git push origin main
```

## 2. Deploy AI Backend (Koyeb - Recommended)
1. Sign in to [Koyeb.com](https://koyeb.com).
2. Click **Create Service** > **GitHub**.
3. Select your `physical-ai-textbook` repository.
4. **Settings**:
   - **Service Name**: `physical-ai-backend`
   - **Work Directory**: `backend`
   - **Build command**: `npm install`
   - **Start command**: `npm start`
5. **Environment Variables**:
   - `OPENAI_API_KEY`: Paste your OpenAI key.
   - `JWT_SECRET`: Paste your secret string.
   - `PORT`: `3001` (Koyeb will map this automatically).
6. Copy your live backend URL (e.g., `https://xxxx-yyyy.koyeb.app`).

## 3. Alternative: Deploy AI Backend (Render)
1. Sign in to [Render.com](https://render.com).
2. Create a **New Web Service**.
3. Connect your GitHub repository.
4. **Settings**:
   - **Root Directory**: `backend`
   - **Build Command**: `npm install`
   - **Start Command**: `npm start`
5. **Environment Variables**:
   - `OPENAI_API_KEY`: Paste your OpenAI key.
   - `JWT_SECRET`: Paste your secret string.
6. Copy your live backend URL (e.g., `https://physical-ai-backend.onrender.com`).

## 3. Deploy Frontend (Netlify)
1. Sign in to [Netlify.com](https://netlify.com).
2. **Add new site** > **Import from GitHub**.
3. Select your repository.
4. **Site Settings**:
   - **Build command**: `npm run build`
   - **Publish directory**: `build`
5. **Environment Variables**:
   - `NEXT_PUBLIC_API_URL`: Paste your **Render Backend URL** (from Step 2).
6. Click **Deploy site**.

## ✅ Project Features Verified
- **Bilingual Interface**: Seamless English/Urdu switching.
- **RAG Chatbot**: AI answers questions based on textbook chapters.
- **User Auth**: Secure Signup/Signin with personalized greetings.
- **Modern Design**: Glassmorphic, dark-themed UI.

**Your Physical AI project is now fully optimized and ready for students!**
