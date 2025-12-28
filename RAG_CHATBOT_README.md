# 🤖 RAG-Powered Chatbot for Physical AI Textbook

## ✅ What's Implemented

### RAG System Features:
- ✅ **67 sections** loaded from book chapters
- ✅ **TF-IDF vectorization** for fast semantic search
- ✅ **Context-aware responses** from actual book content
- ✅ **Source attribution** showing which chapters were referenced
- ✅ **Inline chatbot** integrated below book content
- ✅ **Book-themed UI** matching #646cff color scheme

### Technical Stack:
- **Backend**: Flask + OpenAI API + scikit-learn (TF-IDF)
- **Frontend**: React + TypeScript (already deployed)
- **RAG**: Simple TF-IDF based retrieval (no external DB needed)
- **Book Content**: 6 chapters from `docs/chapters/`

---

## 🚀 Local Testing (Already Running!)

### Backend (RAG Server): ✅ RUNNING
```
http://localhost:5000
- Loaded 67 sections from book
- RAG enabled: true
```

### Frontend: ✅ RUNNING  
```
http://localhost:3000
- Chatbot integrated inline
- Blue theme matching book
```

### Test Commands:
```bash
# Check backend health
curl http://localhost:5000/api/health

# Test chat (requires valid OpenAI API key)
curl -X POST http://localhost:5000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is Physical AI?"}'
```

---

## ⚠️ Current Issue: OpenAI API Key

Backend is working BUT OpenAI API quota exceeded:
```
Error 429: You exceeded your current quota
```

**Solution**: Add valid OpenAI API key in `backend/.env`:
```
OPENAI_API_KEY=sk-proj-your-valid-key-here
```

---

## 📦 Production Deployment to Vercel

### Step 1: Deploy Backend
```bash
cd backend
vercel
```

**Set Environment Variable in Vercel Dashboard:**
```
OPENAI_API_KEY=your-valid-api-key
```

### Step 2: Update Frontend API URL
Edit `frontend/.env.production`:
```
VITE_API_URL=https://your-backend-url.vercel.app
```

### Step 3: Redeploy Frontend
```bash
cd frontend
npm run build
vercel --prod
```

---

## 🎨 Chatbot Features

### User Experience:
1. **Book content** stays at top (unchanged)
2. **Chatbot** appears inline below book
3. **Blue gradient header** matching book theme (#646cff)
4. **Message history** with timestamps
5. **Typing indicator** while AI thinks
6. **Source attribution** showing which chapters were used

### How RAG Works:
1. User asks: "What is Physical AI?"
2. RAG finds **top 3 most relevant sections** from 67 sections
3. Sends relevant context + question to OpenAI
4. AI answers based ONLY on book content
5. Returns answer + source chapters

---

## 📊 Book Content Stats

**Loaded Content:**
- `docs/index.md` - Welcome page
- `docs/preface.md` - Book preface
- `docs/chapters/01-physical-ai.md` - Chapter 1
- `docs/chapters/02-humanoid-robotics.md` - Chapter 2
- `docs/chapters/03-ros2-fundamentals.md` - Chapter 3
- `docs/chapters/04-digital-twin.md` - Chapter 4
- `docs/chapters/05-vla-systems.md` - Chapter 5
- `docs/chapters/06-capstone.md` - Chapter 6

**Total: 67 sections** indexed and searchable!

---

## 🔧 Files Modified/Created

### Backend:
- ✅ `backend/requirements.txt` - Added scikit-learn, numpy
- ✅ `backend/src/utils/simple_rag.py` - RAG processor
- ✅ `backend/src/app_rag.py` - Flask app with RAG
- ✅ `backend/vercel.json` - Vercel config

### Frontend:
- ✅ `frontend/src/components/Chatbot.tsx` - Inline chatbot
- ✅ `frontend/src/components/Chatbot.css` - Blue theme
- ✅ `frontend/.env.development` - Local API URL
- ✅ `frontend/.env.production` - Production API URL

---

## 🎯 Next Steps

### To Use Locally:
1. ✅ Backend running (localhost:5000)
2. ✅ Frontend running (localhost:3000)
3. ❌ Need valid OpenAI API key in `backend/.env`
4. Open browser: http://localhost:3000
5. Chatbot appears below book content
6. Ask questions about Physical AI!

### To Deploy Production:
1. Add valid OpenAI API key
2. Deploy backend to Vercel
3. Update frontend .env.production
4. Redeploy frontend to Vercel
5. Visit: https://physicalaihumanoidbook-6qw6.vercel.app/

---

## 💡 Why RAG?

**Without RAG**: Chatbot gives generic AI responses  
**With RAG**: Chatbot answers from YOUR book content specifically!

**Example:**
- ❌ Generic: "Physical AI is AI in robotics..."
- ✅ RAG: "According to Chapter 1, Physical AI refers to artificial intelligence systems that interact with and manipulate the physical world through embodied agents..."

---

## 📝 Summary

✅ RAG system working locally (67 sections loaded)  
✅ Chatbot integrated inline with book theme  
✅ TF-IDF based semantic search  
✅ Production deployment ready  
❌ Needs valid OpenAI API key to respond  

**Current Status**: Everything coded and running, just need API key!
