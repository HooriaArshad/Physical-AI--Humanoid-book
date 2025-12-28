# Quickstart Guide: Textbook Generation

**Date**: 2025-12-26
**Feature**: 001-textbook-generation
**Purpose**: Local development setup and deployment instructions

---

## Prerequisites

### Required Software
- **Node.js**: 18.x or higher ([download](https://nodejs.org/))
- **Python**: 3.11 or higher ([download](https://www.python.org/))
- **Git**: Latest version ([download](https://git-scm.com/))
- **Code Editor**: VS Code recommended ([download](https://code.visualstudio.com/))

### Free-Tier Accounts (Required)
1. **GitHub**: For repository hosting and GitHub Pages ([signup](https://github.com/signup))
2. **Qdrant Cloud**: Free tier for vector storage ([signup](https://cloud.qdrant.io/))
3. **Neon**: Free tier for PostgreSQL ([signup](https://neon.tech/))
4. **Render** (or Railway): Free tier for backend hosting ([signup](https://render.com/))

---

## Local Setup

### 1. Clone Repository

```bash
git clone https://github.com/<username>/docasuros-book.git
cd docasuros-book
```

### 2. Frontend Setup (Docusaurus)

```bash
# Install Node.js dependencies
npm install

# Start development server (hot reload)
npm start
# Opens http://localhost:3000

# Build static site (for production)
npm run build
# Output: build/ directory

# Serve built site locally
npm run serve
# Opens http://localhost:3000
```

**Expected output**: Docusaurus site with 6 chapters and auto-sidebar

### 3. Backend Setup (FastAPI + RAG)

```bash
# Navigate to backend directory
cd backend

# Create Python virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install Python dependencies
pip install -r requirements.txt
# Installs: FastAPI, sentence-transformers, qdrant-client, psycopg2, etc.

# Return to project root
cd ..
```

### 4. Environment Configuration

Create `.env` file in project root:

```bash
# .env
# Qdrant Cloud
QDRANT_URL=https://<your-cluster>.qdrant.io
QDRANT_API_KEY=<your-api-key>

# Neon PostgreSQL
DATABASE_URL=postgresql://<username>:<password>@<host>.neon.tech/<database>?sslmode=require

# Backend Configuration
BACKEND_PORT=8000
EMBEDDING_MODEL=sentence-transformers/all-MiniLM-L6-v2
CORS_ORIGINS=http://localhost:3000,https://<username>.github.io

# Optional: Rate Limiting
RATE_LIMIT_PER_MINUTE=10
```

**How to get credentials**:

#### Qdrant Cloud
1. Sign up at https://cloud.qdrant.io/
2. Create new cluster (free tier: 1GB)
3. Copy cluster URL and API key from dashboard
4. Paste into `.env` as `QDRANT_URL` and `QDRANT_API_KEY`

#### Neon PostgreSQL
1. Sign up at https://neon.tech/
2. Create new project (free tier: 0.5GB)
3. Copy connection string from dashboard (looks like `postgresql://user:pass@host/db`)
4. Paste into `.env` as `DATABASE_URL`

### 5. Database Initialization

```bash
# Activate Python virtual environment
cd backend
source venv/bin/activate  # or venv\Scripts\activate on Windows

# Create tables in Neon PostgreSQL
python scripts/setup_database.py
# Expected output: "✅ Tables created: chapters, sections, embeddings_metadata, queries, responses"

# Create Qdrant collection
python scripts/setup_qdrant.py
# Expected output: "✅ Collection 'textbook_embeddings' created (384 dims, Cosine distance)"
```

### 6. Ingest Textbook Content

**Prerequisites**: Write 6 chapter markdown files in `docs/chapters/`

```bash
# Generate embeddings and populate Qdrant + Neon
python scripts/ingest_chapters.py

# Expected output:
# Processing Chapter 1: Introduction to Physical AI... ✅
# Processing Chapter 2: Basics of Humanoid Robotics... ✅
# Processing Chapter 3: ROS 2 Fundamentals... ✅
# Processing Chapter 4: Digital Twin Simulation... ✅
# Processing Chapter 5: Vision-Language-Action Systems... ✅
# Processing Chapter 6: Capstone... ✅
#
# Total chunks: 847
# Qdrant points created: 847
# Neon rows inserted: 847
# Time elapsed: 12.4 seconds
```

### 7. Start Backend Server

```bash
# From backend/ directory (with venv activated)
uvicorn src.main:app --reload --port 8000

# Expected output:
# INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
# INFO:     Loading embedding model: all-MiniLM-L6-v2
# INFO:     Model loaded successfully (2.1s)
# INFO:     Connected to Qdrant Cloud
# INFO:     Connected to Neon PostgreSQL
```

**Test backend**:
```bash
# Health check
curl http://localhost:8000/api/health

# Sample query
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}'
```

### 8. Validate RAG Accuracy

```bash
# Run test queries (20+ predefined questions)
python scripts/validate_embeddings.py

# Expected output:
# Query 1: "What is Physical AI?" - ✅ Relevant (confidence: 0.89)
# Query 2: "How to install ROS 2?" - ✅ Relevant (confidence: 0.91)
# Query 3: "What is the weather?" - ✅ Out-of-scope handled correctly
# ...
# Accuracy: 19/20 (95%)
```

---

## Development Workflow

### Adding New Chapters

1. Create markdown file in `docs/chapters/` (e.g., `07-new-chapter.md`)
2. Add frontmatter:
   ```markdown
   ---
   title: "New Chapter Title"
   sidebar_position: 7
   ---
   ```
3. Write content with H2/H3 headings for sections
4. Re-run ingestion: `python scripts/ingest_chapters.py`
5. Verify in Docusaurus: `npm start`

### Updating Existing Content

1. Edit markdown file in `docs/chapters/`
2. Re-run ingestion to update embeddings: `python scripts/ingest_chapters.py --update`
3. Rebuild frontend: `npm run build`

### Testing Locally

```bash
# Terminal 1: Frontend
npm start

# Terminal 2: Backend
cd backend
source venv/bin/activate
uvicorn src.main:app --reload

# Open http://localhost:3000 and test chatbot
```

---

## Deployment

### Frontend (GitHub Pages)

#### Option A: Automated (GitHub Actions)

1. Create `.github/workflows/deploy.yml`:
   ```yaml
   name: Deploy to GitHub Pages

   on:
     push:
       branches: [main]

   jobs:
     deploy:
       runs-on: ubuntu-latest
       steps:
         - uses: actions/checkout@v4
         - uses: actions/setup-node@v4
           with:
             node-version: 18
         - run: npm install
         - run: npm run build
         - uses: peaceiris/actions-gh-pages@v3
           with:
             github_token: ${{ secrets.GITHUB_TOKEN }}
             publish_dir: ./build
   ```

2. Enable GitHub Pages:
   - Go to repository Settings → Pages
   - Source: Deploy from branch
   - Branch: `gh-pages`
   - Save

3. Push to `main` branch → site automatically deploys

4. Access at: `https://<username>.github.io/<repo-name>/`

#### Option B: Manual

```bash
# Build static site
npm run build

# Deploy to gh-pages branch
npm run deploy
# Or use: GIT_USER=<username> npm run deploy
```

### Backend (Render)

1. **Create `render.yaml`** in project root:
   ```yaml
   services:
     - type: web
       name: textbook-rag-backend
       runtime: python
       buildCommand: pip install -r backend/requirements.txt
       startCommand: uvicorn backend.src.main:app --host 0.0.0.0 --port $PORT
       envVars:
         - key: QDRANT_URL
           sync: false
         - key: QDRANT_API_KEY
           sync: false
         - key: DATABASE_URL
           sync: false
         - key: CORS_ORIGINS
           value: https://<username>.github.io
   ```

2. **Deploy to Render**:
   - Go to https://dashboard.render.com/
   - Click "New +" → "Web Service"
   - Connect GitHub repository
   - Render auto-detects `render.yaml`
   - Add environment variables (Qdrant URL, API key, Neon URL)
   - Click "Create Web Service"

3. **Backend URL**: `https://textbook-rag-backend.onrender.com`

4. **Update frontend**: Edit `src/components/ChatbotWidget/index.tsx`:
   ```typescript
   const API_URL = process.env.NODE_ENV === 'production'
     ? 'https://textbook-rag-backend.onrender.com'
     : 'http://localhost:8000';
   ```

### Verify Deployment

```bash
# Test production frontend
curl https://<username>.github.io/<repo-name>/

# Test production backend
curl https://textbook-rag-backend.onrender.com/api/health

# Test RAG query
curl -X POST https://textbook-rag-backend.onrender.com/api/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}'
```

---

## Troubleshooting

### Issue: `npm start` fails with "Cannot find module"
**Solution**: Delete `node_modules/` and `package-lock.json`, then run `npm install`

### Issue: Backend fails with "Model not found"
**Solution**: Ensure sentence-transformers model is downloaded. On first run, model auto-downloads (~80MB). Check internet connection.

### Issue: Qdrant connection error
**Solution**: Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`. Test connection in Qdrant Cloud dashboard.

### Issue: Neon connection timeout
**Solution**: Neon free tier auto-suspends after 5 min inactivity. First query after sleep takes ~1s (cold start). This is expected.

### Issue: RAG returns irrelevant answers
**Solution**: Check embedding quality with `python scripts/validate_embeddings.py`. If accuracy <90%, adjust chunking strategy in `scripts/ingest_chapters.py`.

### Issue: GitHub Pages 404 error
**Solution**: Ensure `docusaurus.config.js` has correct `baseUrl`:
```javascript
module.exports = {
  url: 'https://<username>.github.io',
  baseUrl: '/<repo-name>/',
  // ...
};
```

### Issue: Render free tier exceeded
**Solution**: Check Render dashboard for usage. Free tier: 750 hours/month. If exceeded, consider Railway (500 hours/month) as backup.

---

## Performance Optimization

### Reduce Build Time (<2 minutes)

1. **Incremental builds**: Use `npm start` during development (hot reload)
2. **Optimize images**: Compress images in `docs/assets/` to <200KB each
3. **Cache dependencies**: GitHub Actions caches `node_modules/`

### Reduce RAG Latency (<2 seconds)

1. **Warm start backend**: Configure Render to keep service awake (15 min activity = no cold start)
2. **Optimize embedding**: Batch queries when possible
3. **Cache frequent queries**: Add Redis or in-memory cache (optional, if free tier available)

---

## Maintenance

### Update Dependencies

```bash
# Frontend
npm update

# Backend
cd backend
pip install --upgrade -r requirements.txt
```

### Monitor Free-Tier Usage

- **Qdrant**: Check dashboard for storage (<1GB) and request count (<60/min)
- **Neon**: Check dashboard for storage (<0.5GB) and compute hours
- **Render**: Check dashboard for compute hours (<750/month)

### Cleanup Old Queries (Reduce Neon Storage)

```sql
-- Archive queries older than 90 days
DELETE FROM queries WHERE timestamp < NOW() - INTERVAL '90 days';
```

---

## Next Steps

1. ✅ Local setup complete → Test chatbot at http://localhost:3000
2. ⏳ Write 6 chapter markdown files in `docs/chapters/`
3. ⏳ Run ingestion pipeline (`python scripts/ingest_chapters.py`)
4. ⏳ Deploy frontend to GitHub Pages
5. ⏳ Deploy backend to Render
6. ⏳ Run `/sp.tasks` to generate implementation task list

---

**Quickstart Status**: ✅ Complete (ready for implementation)
