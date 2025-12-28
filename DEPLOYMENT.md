# Deployment Guide - Physical AI Textbook with Chatbot

## 🚀 Quick Setup

### Step 1: Deploy Backend (Flask API) to Vercel

1. **Install Vercel CLI** (agar installed nahi hai):
```bash
npm install -g vercel
```

2. **Backend deploy karo**:
```bash
cd backend
vercel
```

3. **Follow the prompts**:
   - Login to Vercel
   - Link to existing project or create new
   - Accept default settings

4. **Environment variable set karo** (Vercel dashboard):
   - Go to: Project Settings → Environment Variables
   - Add: `OPENAI_API_KEY` = your-openai-api-key
   - Redeploy: `vercel --prod`

5. **Backend URL copy karo** (e.g., `https://your-backend.vercel.app`)

---

### Step 2: Update Frontend with Backend URL

1. **Edit `.env.production` file**:
```bash
cd frontend
```

Edit `frontend/.env.production`:
```
VITE_API_URL=https://your-backend.vercel.app
```
(Replace with your actual backend URL from Step 1)

2. **Rebuild frontend**:
```bash
npm run build
```

---

### Step 3: Deploy Frontend to Vercel

**Option A: Using Vercel CLI**
```bash
cd frontend
vercel --prod
```

**Option B: Using Vercel Dashboard**
1. Go to https://vercel.com/dashboard
2. Import your GitHub repository
3. Set build settings:
   - Framework Preset: Vite
   - Build Command: `npm run build`
   - Output Directory: `dist`
4. Add environment variable:
   - `VITE_API_URL` = your-backend-url
5. Deploy!

---

### Step 4: Test Production Site

1. Visit your production URL: `https://physicalaihumanoidbook-6qw6.vercel.app/`
2. Click the floating blue chat button (💬) at bottom-right
3. Ask a question to test the chatbot
4. Backend should respond with AI-generated answers

---

## 🔧 Environment Variables Summary

### Backend (Vercel Project Settings)
```
OPENAI_API_KEY=sk-proj-xxx...
```

### Frontend (Vercel Project Settings)
```
VITE_API_URL=https://your-backend.vercel.app
```

---

## 📝 File Structure

```
backend/
├── src/app.py           # Flask API
├── requirements.txt     # Python dependencies
├── vercel.json         # Vercel config
└── .gitignore

frontend/
├── src/
│   └── components/Chatbot.tsx  # Uses VITE_API_URL
├── .env.development    # Local: http://localhost:5000
├── .env.production     # Production: your-backend-url
└── package.json
```

---

## 🐛 Troubleshooting

### Backend not responding
- Check Vercel logs: `vercel logs`
- Verify OPENAI_API_KEY is set in Vercel dashboard
- Test health endpoint: `https://your-backend.vercel.app/api/health`

### Frontend can't reach backend
- Check browser console for CORS errors
- Verify VITE_API_URL in Vercel environment variables
- Make sure you rebuilt after changing .env.production

### CORS Issues
Backend already has `CORS(app)` enabled in `app.py:7`

---

## 🎨 Features

✅ Floating blue chatbot button (matches book theme #646cff)
✅ Click to show/hide chatbot
✅ AI-powered responses using OpenAI GPT-3.5
✅ Book-themed UI design
✅ Production-ready with environment variables

---

## 🔗 Your URLs

- Frontend (current): https://physicalaihumanoidbook-6qw6.vercel.app/
- Backend (to deploy): https://your-backend.vercel.app/

After deployment, update this file with actual URLs!
