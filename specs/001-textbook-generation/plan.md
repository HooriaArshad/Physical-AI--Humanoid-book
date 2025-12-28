# Implementation Plan: Textbook Generation

**Branch**: `001-textbook-generation` | **Date**: 2025-12-26 | **Spec**: [spec.md](./spec.md)

**Input**: Feature specification from `/specs/001-textbook-generation/spec.md`

## Summary

Build an AI-native textbook for Physical AI & Humanoid Robotics with 6 chapters and integrated RAG chatbot. System generates static Docusaurus site with auto-sidebar navigation, embeds chapter content into Qdrant vector database, and provides chatbot interface that answers questions exclusively from textbook content. Free-tier architecture using Qdrant Cloud, Neon PostgreSQL, and GitHub Pages hosting.

## Technical Context

**Language/Version**: Python 3.11+ (RAG backend), Node.js 18+ (Docusaurus frontend), TypeScript 5.0+

**Primary Dependencies**:
- Frontend: Docusaurus 3.x, React 18, TypeScript
- Backend: FastAPI 0.109+, sentence-transformers 2.3+, qdrant-client 1.7+, psycopg2 2.9+
- Embedding: all-MiniLM-L6-v2 (sentence-transformers model, 80MB)

**Storage**:
- Vector: Qdrant Cloud free tier (1GB, 100k vectors)
- Metadata: Neon PostgreSQL free tier (0.5GB, serverless)
- Static: GitHub Pages (unlimited for public repos)

**Testing**: pytest (backend), Jest (frontend components), Cypress (E2E), markdownlint (content)

**Target Platform**: Web (static site), Python backend on free-tier hosting (Render/Railway)

**Project Type**: Web application (static frontend + REST API backend)

**Performance Goals**:
- Build time: <2 minutes (Docusaurus)
- RAG response: <2 seconds end-to-end
- Page load: <3 seconds on 3G
- Embedding generation: <500ms per query (CPU)
- Vector search: <200ms

**Constraints**:
- No GPU usage (CPU-only embeddings)
- Free-tier limits: Qdrant 1GB, Neon 0.5GB, no paid APIs
- Embedding model <100MB
- Backend memory <512MB
- 6 chapters exactly (no scope expansion)

**Scale/Scope**:
- 6 chapters, ~15-30 min read each (90-180 min total)
- Estimated 50-100 markdown pages
- ~500-1000 text chunks for embedding
- <500MB total vector data
- Target: <1000 queries/day (free-tier safe)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Principle I: Simplicity First
- Static site generation (no complex server-side rendering)
- Single-purpose components (sidebar, chatbot, text-selection)
- Minimal dependencies (Docusaurus + FastAPI)
- No premature abstraction (direct Qdrant/Neon access, no ORM)

### ✅ Principle II: Accuracy and Quality
- Code examples tested in CI
- RAG answers cite sources (chapter/section)
- Markdown linting enforced
- Build failures block deployment

### ✅ Principle III: Minimalism in Scope
- Exactly 6 chapters (enforced in build)
- No features beyond spec (no analytics, no auth, no progress tracking)
- RAG strictly textbook-content (no external knowledge)

### ✅ Principle IV: Free-Tier Architecture (NON-NEGOTIABLE)
- Qdrant Cloud free tier (1GB)
- Neon free tier (0.5GB)
- GitHub Pages (free static hosting)
- sentence-transformers CPU inference (no GPU cost)
- FastAPI on free-tier hosting (Render/Railway)

### ✅ Principle V: Fast Builds and Deployment
- Docusaurus optimized build (<2 min)
- GitHub Actions CI/CD (<5 min total)
- Single command deployment (git push)

### ✅ Principle VI: RAG Integrity
- Embeddings only from 6 chapter markdown files
- Out-of-scope refusal mechanism
- Citation mandatory in responses
- No external knowledge injection

### ✅ Principle VII: Testing and Validation
- Build success gate (exit code 0)
- RAG accuracy testing (20+ query test set)
- Link validation (no 404s)
- Accessibility audit (WCAG AA)

**Constitution Compliance**: ✅ PASS (all 7 principles satisfied)

## Project Structure

### Documentation (this feature)

```text
specs/001-textbook-generation/
├── plan.md              # This file (/sp.plan output)
├── spec.md              # Feature specification
├── research.md          # Phase 0 research findings
├── data-model.md        # Phase 1 data entities
├── quickstart.md        # Phase 1 local setup guide
├── contracts/           # Phase 1 API specifications
│   ├── rag-api.yaml     # FastAPI endpoints (OpenAPI 3.0)
│   └── embeddings.yaml  # Embedding service spec
├── checklists/
│   └── requirements.md  # Spec quality validation
└── tasks.md             # Phase 2 (/sp.tasks output - NOT created yet)
```

### Source Code (repository root)

```text
# Web application structure (static frontend + backend)

# Frontend (Docusaurus static site)
docs/
├── intro.md                      # Welcome/landing page
├── chapters/
│   ├── 01-physical-ai.md         # Chapter 1: Introduction to Physical AI
│   ├── 02-humanoid-robotics.md   # Chapter 2: Basics of Humanoid Robotics
│   ├── 03-ros2-fundamentals.md   # Chapter 3: ROS 2 Fundamentals
│   ├── 04-digital-twin.md        # Chapter 4: Digital Twin Simulation
│   ├── 05-vla-systems.md         # Chapter 5: Vision-Language-Action
│   └── 06-capstone.md            # Chapter 6: Capstone Project
└── assets/
    └── images/                   # Optimized images (<200KB each)

docusaurus.config.js              # Docusaurus configuration
sidebars.js                       # Auto-generated sidebar config
package.json                      # Frontend dependencies

src/
├── components/
│   ├── ChatbotWidget/            # RAG chatbot UI component
│   │   ├── index.tsx
│   │   ├── ChatInterface.tsx
│   │   └── styles.module.css
│   └── TextSelection/            # Text-to-question UI
│       ├── index.tsx
│       ├── SelectionPopup.tsx
│       └── styles.module.css
├── theme/                        # Docusaurus theme customization
└── css/
    └── custom.css                # Global styles

# Backend (RAG system)
backend/
├── src/
│   ├── main.py                   # FastAPI app entry
│   ├── api/
│   │   ├── routes.py             # API endpoints (/query, /health)
│   │   └── models.py             # Pydantic request/response models
│   ├── services/
│   │   ├── embeddings.py         # sentence-transformers wrapper
│   │   ├── vectorstore.py        # Qdrant client operations
│   │   ├── database.py           # Neon PostgreSQL connection
│   │   └── rag.py                # RAG orchestration logic
│   ├── config.py                 # Environment config
│   └── utils/
│       ├── chunk.py              # Markdown chunking logic
│       └── citations.py          # Citation formatting
├── tests/
│   ├── test_embeddings.py        # Embedding generation tests
│   ├── test_rag.py               # RAG retrieval accuracy tests
│   └── test_api.py               # API endpoint tests
├── requirements.txt              # Python dependencies
└── .env.example                  # Environment variables template

# Ingestion pipeline (one-time setup)
scripts/
├── ingest_chapters.py            # Embed chapters into Qdrant
├── setup_database.py             # Initialize Neon schema
└── validate_embeddings.py        # Test retrieval accuracy

# CI/CD
.github/
└── workflows/
    ├── build.yml                 # Docusaurus build + deploy
    ├── backend-test.yml          # Backend tests
    └── lint.yml                  # Markdown + code linting

# Configuration
.env                              # Local environment (gitignored)
.markdownlintrc                   # Markdown linting rules
README.md                         # Project setup instructions
```

**Structure Decision**: Web application with static frontend (Docusaurus) and REST API backend (FastAPI). Frontend deployed to GitHub Pages, backend to free-tier hosting (Render/Railway). Separation enables independent scaling and deployment. Ingestion scripts run once during setup to populate vector database.

## Complexity Tracking

> **No violations detected** - Constitution check passed all 7 principles. Free-tier architecture is simple, minimal, and aligns with educational goals.

---

## Phase 0: Research & Technology Validation

**Purpose**: Resolve technical unknowns, validate free-tier feasibility, establish best practices

**Key Tasks**:
- Research Docusaurus auto-sidebar configuration for markdown structure
- Validate sentence-transformers CPU performance on target chapters
- Test Qdrant Cloud free tier capacity (1GB, 100k vectors) with sample data
- Verify Neon free tier sufficient for metadata (chapter/section/chunk mappings)
- Evaluate free-tier hosting options for FastAPI (Render vs Railway vs others)
- Research text selection UI patterns (browser APIs, accessibility)
- Test end-to-end RAG latency: embedding (CPU) → Qdrant search → response generation

**Expected Outputs**: `research.md` with decisions, rationale, and alternatives for each unknown

---

## Phase 1: Design & Contracts

**Purpose**: Define data model, API contracts, and quickstart workflow

**Prerequisites**: `research.md` complete (all NEEDS CLARIFICATION resolved)

**Key Tasks**:

### Data Model
- Define Chapter entity (id, title, slug, order, reading_time, content, sections[])
- Define Section entity (id, chapter_id, title, slug, content, heading_level)
- Define Embedding entity (id, vector, chunk_text, chapter_id, section_id, metadata)
- Define Query entity (id, text, timestamp, user_ip_hash, retrieved_chunk_ids[])
- Define Response entity (id, query_id, answer_text, citations[], confidence_score)
- Document Qdrant collection schema (vector_size=384, distance=Cosine)
- Document Neon table schemas (chapters, sections, embeddings_metadata, queries, responses)

### API Contracts
- **POST /api/query**: Submit question, return answer with citations
  - Request: `{question: string, context?: string}`
  - Response: `{answer: string, citations: [{chapter, section, page}], confidence: float}`
- **GET /api/health**: Backend health check
- **POST /api/embed**: (Internal) Generate embedding for text chunk
  - Request: `{text: string}`
  - Response: `{vector: float[], model: string}`
- OpenAPI 3.0 spec output to `contracts/rag-api.yaml`

### Quickstart Guide
- Local development setup (Node.js, Python, dependencies)
- Qdrant Cloud account setup (free tier)
- Neon account setup (free tier, connection string)
- Environment variable configuration (.env template)
- Run ingestion pipeline (embed 6 chapters)
- Start Docusaurus dev server (`npm start`)
- Start FastAPI backend (`uvicorn main:app --reload`)
- Test chatbot locally

**Expected Outputs**: `data-model.md`, `contracts/rag-api.yaml`, `contracts/embeddings.yaml`, `quickstart.md`

---

## Phase 2: Implementation Planning (Deferred to /sp.tasks)

**Purpose**: Generate task list for parallel execution

**Prerequisites**: Phase 0 + Phase 1 complete, constitution re-validated

**Key Tasks**: Run `/sp.tasks` command to generate `tasks.md` with:
- Setup phase (project init, dependencies)
- Foundational phase (Docusaurus config, database schema)
- User Story 1 (P1): Read Textbook Content
- User Story 4 (P1): Auto-Sidebar Navigation
- User Story 2 (P2): RAG Chatbot
- User Story 3 (P3): Text Selection Feature
- Optional: User Story 5 (P4) Urdu translation
- Optional: User Story 6 (P5) Personalized welcome
- Polish phase (testing, deployment, validation)

**Expected Outputs**: `tasks.md` (NOT generated by this command - run `/sp.tasks`)

---

## Deployment Strategy

### Frontend (GitHub Pages)
1. Build Docusaurus static site (`npm run build`)
2. GitHub Actions workflow triggers on push to `main`
3. Deploy `build/` directory to `gh-pages` branch
4. Accessible at `https://<username>.github.io/<repo-name>/`

### Backend (Free-Tier Hosting)
1. Choose hosting: Render (recommended) or Railway
2. Connect GitHub repo to hosting platform
3. Configure environment variables (Qdrant API key, Neon connection string)
4. Auto-deploy on push to `main`
5. Backend URL: `https://<app-name>.onrender.com/` or similar

### Ingestion Pipeline (One-Time Setup)
1. Run `scripts/setup_database.py` (create Neon tables)
2. Run `scripts/ingest_chapters.py` (embed 6 chapters into Qdrant)
3. Validate with `scripts/validate_embeddings.py` (test queries)

---

## Risk Mitigation

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Free-tier rate limits exceeded | Medium | High | Implement request queuing, exponential backoff, graceful degradation |
| Embedding quality insufficient for educational Q&A | Low | High | Test with 20+ query set early (Phase 0), iterate on chunking strategy |
| Build time exceeds 2 minutes | Low | Medium | Profile Docusaurus build, optimize assets, use incremental builds |
| Backend hosting costs exceed free tier | Medium | High | Monitor usage, implement rate limiting, choose provider with generous free tier |
| RAG hallucination (answers outside textbook) | Medium | High | Strict similarity threshold, cite-only-from-chunks enforcement, refusal mechanism |
| Text selection feature breaks on mobile | Low | Medium | Test on iOS/Android browsers, use standard selection APIs |

---

## Success Validation Checklist

### Build Success
- [ ] `npm run build` exits with code 0
- [ ] Build completes in <2 minutes
- [ ] All 6 chapters render correctly
- [ ] Sidebar auto-generated from markdown structure
- [ ] No broken internal links

### RAG Accuracy
- [ ] 20+ test queries achieve >90% retrieval accuracy
- [ ] All responses cite specific chapter/section
- [ ] Out-of-scope queries refused 100% of time
- [ ] End-to-end latency <2 seconds

### Free-Tier Compliance
- [ ] Qdrant storage <1GB (check dashboard)
- [ ] Neon storage <0.5GB (check dashboard)
- [ ] No paid API calls made
- [ ] Backend memory <512MB (check hosting metrics)

### Performance
- [ ] Homepage loads <3 seconds on 3G throttle
- [ ] Chatbot response <2 seconds
- [ ] Text selection works on all paragraphs/code blocks

### Accessibility
- [ ] WCAG AA audit passes (Lighthouse or axe)
- [ ] Keyboard navigation works (Tab, Enter)
- [ ] Screen reader compatible (ARIA labels)

---

## Next Steps

1. ✅ Constitution check passed - proceed to Phase 0
2. ⏳ Run research tasks (validate technology choices)
3. ⏳ Generate `research.md` with findings
4. ⏳ Run Phase 1 (data model, contracts, quickstart)
5. ⏳ Run `/sp.tasks` to generate implementation task list
6. ⏳ Begin implementation (User Story 1 + 4 MVP)

---

**Plan Status**: ✅ Complete (Phases 0-1 defined, ready for execution)

**Command**: This plan stops after Phase 1 planning. Run `/sp.tasks` next to generate implementation tasks.
