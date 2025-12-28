# Data Model: Textbook Generation

**Date**: 2025-12-26
**Feature**: 001-textbook-generation
**Purpose**: Define entities, relationships, and storage schemas

---

## Entity Definitions

### Chapter

**Description**: A complete learning module covering one major topic in the textbook

**Attributes**:
- `id` (UUID): Unique identifier
- `order` (Integer): Sequential position (1-6)
- `title` (String, max 200 chars): Display name (e.g., "Introduction to Physical AI")
- `slug` (String, max 100 chars): URL-safe identifier (e.g., "01-physical-ai")
- `markdown_path` (String): Relative path to source file (e.g., "docs/chapters/01-physical-ai.md")
- `estimated_reading_time` (Integer): Minutes (15-30)
- `created_at` (Timestamp): When chapter was added
- `updated_at` (Timestamp): Last modification time

**Relationships**:
- Has many `Section` (one-to-many)
- Has many `Embedding` via sections (indirect)

**Validation Rules**:
- `order` must be 1-6 (exactly 6 chapters)
- `slug` must be unique
- `estimated_reading_time` must be 15-30
- `title` must not be empty

**Storage**: Neon PostgreSQL table

---

### Section

**Description**: A subsection within a chapter, derived from H2/H3 markdown headings

**Attributes**:
- `id` (UUID): Unique identifier
- `chapter_id` (UUID, foreign key): Parent chapter
- `title` (String, max 200 chars): Section heading text
- `slug` (String, max 100 chars): URL-safe anchor (e.g., "ros2-installation")
- `heading_level` (Integer): 2 or 3 (H2/H3)
- `order` (Integer): Position within chapter
- `content` (Text): Raw markdown content for this section
- `created_at` (Timestamp): When section was parsed
- `updated_at` (Timestamp): Last modification time

**Relationships**:
- Belongs to `Chapter` (many-to-one)
- Has many `Embedding` (one-to-many)

**Validation Rules**:
- `heading_level` must be 2 or 3
- `slug` must be unique within chapter
- `content` must not be empty

**Storage**: Neon PostgreSQL table

---

### Embedding

**Description**: Vector representation of a text chunk from the textbook

**Attributes**:
- `id` (UUID): Unique identifier
- `vector` (Float array, 384 dimensions): Embedding from all-MiniLM-L6-v2
- `chunk_text` (Text, max 1000 chars): Original text this embedding represents
- `chapter_id` (UUID): Source chapter
- `section_id` (UUID, nullable): Source section (null if chapter-level)
- `chunk_index` (Integer): Position within section (for ordering)
- `metadata` (JSONB): Additional context (e.g., `{heading: "ROS 2 Core Concepts", page: 3}`)
- `created_at` (Timestamp): When embedding was generated

**Relationships**:
- Belongs to `Chapter` (many-to-one)
- Belongs to `Section` (many-to-one, nullable)
- Referenced by `Response` via `retrieved_chunks` (many-to-many through join)

**Validation Rules**:
- `vector` must have exactly 384 dimensions
- `chunk_text` length: 100-1000 characters (optimal for semantic search)
- `chapter_id` must reference valid chapter

**Storage**:
- **Vectors**: Qdrant Cloud (collection: `textbook_embeddings`)
- **Metadata**: Neon PostgreSQL table (for relational queries)

**Qdrant Collection Schema**:
```json
{
  "collection_name": "textbook_embeddings",
  "vector_size": 384,
  "distance": "Cosine",
  "payload_schema": {
    "chunk_id": "uuid",
    "chapter_id": "uuid",
    "section_id": "uuid",
    "chunk_text": "text",
    "metadata": "object"
  }
}
```

---

### Query

**Description**: User question submitted to the RAG chatbot

**Attributes**:
- `id` (UUID): Unique identifier
- `text` (Text, max 5000 chars): User's question
- `timestamp` (Timestamp): When query was submitted
- `user_ip_hash` (String, 64 chars): SHA-256 of IP (privacy-preserving, for rate limiting)
- `retrieved_chunk_ids` (UUID array): Top chunks returned from vector search
- `response_id` (UUID, nullable foreign key): Associated response (null if failed)
- `latency_ms` (Integer): Time to generate response (for monitoring)
- `status` (Enum): `success`, `error`, `out_of_scope`

**Relationships**:
- Has one `Response` (one-to-one)
- References many `Embedding` via `retrieved_chunk_ids` (many-to-many)

**Validation Rules**:
- `text` must not be empty
- `text` max length 5000 characters (truncate with warning)
- `user_ip_hash` must be SHA-256 format (64 hex chars)

**Storage**: Neon PostgreSQL table

---

### Response

**Description**: AI-generated answer to a user query

**Attributes**:
- `id` (UUID): Unique identifier
- `query_id` (UUID, foreign key): Associated query
- `answer_text` (Text): Generated response
- `citations` (JSONB array): References to textbook sections
  - Format: `[{chapter: "Chapter 1", section: "Physical AI Intro", url: "/chapters/01-physical-ai#intro"}]`
- `confidence_score` (Float, 0-1): Retrieval confidence (average similarity of top chunks)
- `model_version` (String): Embedding model used (e.g., "all-MiniLM-L6-v2")
- `created_at` (Timestamp): When response was generated

**Relationships**:
- Belongs to `Query` (one-to-one)

**Validation Rules**:
- `answer_text` must not be empty
- `citations` must be valid JSON array
- `confidence_score` must be 0-1
- Each citation must have `chapter`, `section`, and `url` fields

**Storage**: Neon PostgreSQL table

---

## Entity Relationships Diagram

```text
┌─────────────┐
│   Chapter   │
│             │
│ - id        │
│ - order     │
│ - title     │
│ - slug      │
└──────┬──────┘
       │ 1
       │
       │ N
┌──────▼──────┐
│   Section   │
│             │
│ - id        │
│ - chapter_id│
│ - title     │
│ - slug      │
└──────┬──────┘
       │ 1
       │
       │ N
┌──────▼──────────────┐
│    Embedding        │
│                     │
│ - id                │
│ - vector (Qdrant)   │
│ - chunk_text        │
│ - chapter_id        │
│ - section_id        │
└──────┬──────────────┘
       │ N
       │
       │ M (via retrieved_chunk_ids)
┌──────▼──────┐      ┌─────────────┐
│    Query    │ 1───1│  Response   │
│             │      │             │
│ - id        │      │ - id        │
│ - text      │      │ - query_id  │
│ - timestamp │      │ - answer    │
│ - chunk_ids │      │ - citations │
└─────────────┘      └─────────────┘
```

---

## Storage Implementation

### Neon PostgreSQL Tables

#### `chapters` Table
```sql
CREATE TABLE chapters (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    order_num INTEGER NOT NULL UNIQUE CHECK (order_num BETWEEN 1 AND 6),
    title VARCHAR(200) NOT NULL,
    slug VARCHAR(100) NOT NULL UNIQUE,
    markdown_path VARCHAR(500) NOT NULL,
    estimated_reading_time INTEGER NOT NULL CHECK (estimated_reading_time BETWEEN 15 AND 30),
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_chapters_order ON chapters(order_num);
CREATE INDEX idx_chapters_slug ON chapters(slug);
```

#### `sections` Table
```sql
CREATE TABLE sections (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id UUID NOT NULL REFERENCES chapters(id) ON DELETE CASCADE,
    title VARCHAR(200) NOT NULL,
    slug VARCHAR(100) NOT NULL,
    heading_level INTEGER NOT NULL CHECK (heading_level IN (2, 3)),
    order_num INTEGER NOT NULL,
    content TEXT NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    UNIQUE (chapter_id, slug)
);

CREATE INDEX idx_sections_chapter ON sections(chapter_id);
CREATE INDEX idx_sections_slug ON sections(chapter_id, slug);
```

#### `embeddings_metadata` Table
```sql
CREATE TABLE embeddings_metadata (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    qdrant_point_id UUID NOT NULL UNIQUE, -- References Qdrant point ID
    chunk_text TEXT NOT NULL CHECK (char_length(chunk_text) BETWEEN 100 AND 1000),
    chapter_id UUID NOT NULL REFERENCES chapters(id) ON DELETE CASCADE,
    section_id UUID REFERENCES sections(id) ON DELETE SET NULL,
    chunk_index INTEGER NOT NULL,
    metadata JSONB,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_embeddings_chapter ON embeddings_metadata(chapter_id);
CREATE INDEX idx_embeddings_section ON embeddings_metadata(section_id);
CREATE INDEX idx_embeddings_qdrant ON embeddings_metadata(qdrant_point_id);
```

#### `queries` Table
```sql
CREATE TABLE queries (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    text TEXT NOT NULL CHECK (char_length(text) > 0 AND char_length(text) <= 5000),
    timestamp TIMESTAMPTZ NOT NULL DEFAULT NOW(),
    user_ip_hash VARCHAR(64) NOT NULL, -- SHA-256 hash
    retrieved_chunk_ids UUID[] NOT NULL,
    response_id UUID REFERENCES responses(id) ON DELETE SET NULL,
    latency_ms INTEGER,
    status VARCHAR(20) NOT NULL CHECK (status IN ('success', 'error', 'out_of_scope'))
);

CREATE INDEX idx_queries_timestamp ON queries(timestamp);
CREATE INDEX idx_queries_user_hash ON queries(user_ip_hash, timestamp); -- For rate limiting
```

#### `responses` Table
```sql
CREATE TABLE responses (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    query_id UUID NOT NULL UNIQUE REFERENCES queries(id) ON DELETE CASCADE,
    answer_text TEXT NOT NULL,
    citations JSONB NOT NULL, -- Array of {chapter, section, url}
    confidence_score REAL NOT NULL CHECK (confidence_score BETWEEN 0 AND 1),
    model_version VARCHAR(50) NOT NULL,
    created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_responses_query ON responses(query_id);
```

---

### Qdrant Collection

**Collection Name**: `textbook_embeddings`

**Configuration**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

client.create_collection(
    collection_name="textbook_embeddings",
    vectors_config=VectorParams(size=384, distance=Distance.COSINE),
)
```

**Point Structure**:
```json
{
  "id": "uuid-string",
  "vector": [0.1, -0.3, 0.5, ...],  // 384 floats
  "payload": {
    "chunk_id": "uuid-string",
    "chapter_id": "uuid-string",
    "section_id": "uuid-string",
    "chunk_text": "ROS 2 is a middleware framework...",
    "metadata": {
      "heading": "ROS 2 Core Concepts",
      "chapter_title": "ROS 2 Fundamentals",
      "section_title": "Introduction"
    }
  }
}
```

---

## Data Consistency Rules

1. **Chapter Order**: Must be sequential 1-6, no gaps
2. **Cascade Deletes**: Deleting chapter removes all sections and embeddings
3. **Orphan Prevention**: Embedding cannot exist without valid chapter_id
4. **Citation Integrity**: Response citations must reference existing chapters/sections
5. **Timestamp Tracking**: All entities track creation time for audit

---

## Data Migration Strategy

1. **Initial Setup**: Run `scripts/setup_database.py` to create Neon tables
2. **Ingestion**: Run `scripts/ingest_chapters.py` to:
   - Parse markdown files
   - Create chapter/section records in Neon
   - Generate embeddings using sentence-transformers
   - Store vectors in Qdrant
   - Store metadata in Neon `embeddings_metadata` table
3. **Validation**: Run `scripts/validate_embeddings.py` to test retrieval

---

## Performance Considerations

- **Indexes**: Created on foreign keys, slugs, and timestamps for fast lookups
- **Qdrant Search**: Cosine distance optimized for semantic similarity
- **Query Logs**: Periodically archive old queries (>90 days) to keep table small
- **Neon Auto-Suspend**: Database sleeps after 5 min inactivity (free tier), ~1s cold start

---

**Data Model Status**: ✅ Complete (ready for implementation)
