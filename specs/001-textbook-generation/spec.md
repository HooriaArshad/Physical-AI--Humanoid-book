# Feature Specification: Textbook Generation

**Feature Branch**: `001-textbook-generation`
**Created**: 2025-12-26
**Status**: Draft
**Input**: User description: "Build AI-native textbook with RAG chatbot for Physical AI & Humanoid Robotics"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Read Textbook Content (Priority: P1)

As a student learning Physical AI and Humanoid Robotics, I want to read clear, concise textbook chapters that cover foundational concepts, so I can understand the field without being overwhelmed by complexity.

**Why this priority**: This is the core value proposition - delivering educational content. Without readable chapters, there is no textbook. This is the foundational MVP.

**Independent Test**: Deploy static site with 6 chapters. User navigates through chapters using sidebar, reads content, and verifies all pages load correctly without broken links or formatting issues.

**Acceptance Scenarios**:

1. **Given** a user visits the textbook homepage, **When** they click on "Introduction to Physical AI" in the sidebar, **Then** the chapter loads within 3 seconds and displays readable formatted text with headings and code examples
2. **Given** a user is reading Chapter 3 (ROS 2 Fundamentals), **When** they click "Next" or select Chapter 4 from the sidebar, **Then** they navigate to the Digital Twin chapter seamlessly
3. **Given** a user on mobile device visits any chapter, **When** the page loads, **Then** the content is responsive and readable without horizontal scrolling
4. **Given** a user clicks a code example in any chapter, **When** they copy the code, **Then** the formatting is preserved and code runs without syntax errors

---

### User Story 2 - Ask Questions with RAG Chatbot (Priority: P2)

As a student reading the textbook, I want to ask specific questions about the content and receive accurate answers sourced only from the textbook, so I can clarify concepts without leaving the page or getting incorrect external information.

**Why this priority**: Differentiates this from static textbooks and provides AI-native learning experience. Enhances understanding through interactive Q&A. Depends on textbook content existing first (P1).

**Independent Test**: With textbook deployed, user opens chatbot interface, asks "What is Physical AI?", and receives answer with citation to Chapter 1. Verify answer quotes/paraphrases textbook content only.

**Acceptance Scenarios**:

1. **Given** a user is reading Chapter 2 (Humanoid Robotics), **When** they select the text "degrees of freedom" and click "Ask AI", **Then** the chatbot opens with the selected text pre-populated and responds with relevant explanation from the textbook
2. **Given** a user types "How do I use Gazebo for simulation?" in the chatbot, **When** the RAG system searches, **Then** it returns answers from Chapter 4 with specific section citations
3. **Given** a user asks "What is the weather today?" (out of scope), **When** the RAG system processes the query, **Then** it responds with "I can only answer questions about Physical AI and Humanoid Robotics content in this textbook"
4. **Given** a user asks a question about Vision-Language-Action systems, **When** the response is generated, **Then** it includes clickable references to the relevant chapter sections

---

### User Story 3 - Select Text to Ask Contextual Questions (Priority: P3)

As a student reading complex technical terms or concepts, I want to select any text on the page and immediately ask the AI about it, so I can get instant clarification without typing the full question.

**Why this priority**: Improves UX by reducing friction for contextual questions. Enhances P2 by making RAG access more intuitive. Nice-to-have enhancement, not critical for MVP.

**Independent Test**: User highlights "sensor fusion" in Chapter 5, right-clicks or uses button to trigger "Ask AI about this", and chatbot opens with context-aware query pre-filled.

**Acceptance Scenarios**:

1. **Given** a user is reading Chapter 1, **When** they highlight the text "embodied intelligence", **Then** a small "Ask AI" button appears next to the selection
2. **Given** a user clicks the "Ask AI" button on selected text, **When** the chatbot opens, **Then** the question field contains "Tell me more about [selected text]" and the AI provides relevant context from the textbook
3. **Given** a user selects text that spans multiple paragraphs, **When** they trigger "Ask AI", **Then** the system handles the selection gracefully (truncate if needed) and asks a coherent question

---

### User Story 4 - Navigate Auto-Generated Sidebar (Priority: P1)

As a student using the textbook, I want an automatically generated sidebar that shows all chapters and sections, so I can quickly jump to any topic without manually searching.

**Why this priority**: Essential for usability. Without navigation, multi-chapter textbook is unusable. Core to Docusaurus functionality and MVP.

**Independent Test**: Build textbook and verify sidebar automatically displays all 6 chapters with nested sections based on markdown headings.

**Acceptance Scenarios**:

1. **Given** the textbook has 6 markdown chapter files, **When** the site is built, **Then** the sidebar displays all 6 chapters in order with correct titles
2. **Given** Chapter 3 (ROS 2) has sections "Installation", "Core Concepts", "Publishers and Subscribers", **When** a user clicks Chapter 3 in the sidebar, **Then** these subsections appear as expandable/nested items
3. **Given** a user is viewing Chapter 5, **When** they look at the sidebar, **Then** the current chapter is visually highlighted
4. **Given** the sidebar contains 50+ items (chapters + sections), **When** a user scrolls the page, **Then** the sidebar remains accessible (sticky or scrollable)

---

### User Story 5 - Translate Content to Urdu (Priority: P4 - Optional)

As an Urdu-speaking student, I want to view the entire textbook in Urdu, so I can learn in my native language.

**Why this priority**: Increases accessibility for Urdu speakers but not critical for initial launch. Optional enhancement.

**Independent Test**: User switches language toggle to Urdu, and all chapter content displays in Urdu translation while maintaining formatting and code examples.

**Acceptance Scenarios**:

1. **Given** a user visits the homepage, **When** they click the language switcher and select "Urdu", **Then** all navigation, chapter titles, and body text display in Urdu
2. **Given** a code example in Python appears in the English version, **When** the user switches to Urdu, **Then** the code remains in English but comments and explanations are translated
3. **Given** a user asks a question in Urdu using the RAG chatbot, **When** the system processes the query, **Then** it responds in Urdu with content from the Urdu-translated chapters

---

### User Story 6 - View Personalized Welcome Chapter (Priority: P5 - Optional)

As a new user visiting the textbook for the first time, I want a personalized welcome chapter that introduces me to the book's structure and my learning path, so I feel oriented and motivated.

**Why this priority**: Nice-to-have for onboarding UX. Lowest priority - doesn't affect core learning value.

**Independent Test**: First-time user visits site and sees custom welcome page with overview of chapters, learning objectives, and estimated time commitments.

**Acceptance Scenarios**:

1. **Given** a user visits the textbook homepage for the first time, **When** the page loads, **Then** a "Welcome" chapter appears as the first item explaining the book structure
2. **Given** the welcome chapter describes each of the 6 chapters, **When** a user reads it, **Then** they see a brief summary (2-3 sentences) and estimated reading time for each chapter

---

### Edge Cases

- **What happens when RAG chatbot receives ambiguous queries?** System should ask clarifying questions or provide the most relevant sections from multiple chapters
- **How does the system handle very long questions (>500 words)?** Truncate with warning and process the first 500 words
- **What if a user asks about content not yet written?** Respond "This topic is not covered in the current textbook version"
- **How does navigation work if a chapter markdown file is missing?** Build process fails with clear error message indicating missing file
- **What if vector database or RAG backend is down?** Display user-friendly error: "AI assistant temporarily unavailable. Please try again later"
- **How are code examples formatted across different screen sizes?** Use responsive code blocks with horizontal scroll for mobile
- **What happens if embedding generation exceeds free-tier rate limits?** Queue requests and process incrementally, or fail gracefully with retry logic
- **How does the system handle special characters or math notation in queries?** Preserve formatting in both query and response using markdown rendering

## Requirements *(mandatory)*

### Functional Requirements

#### Content Structure

- **FR-001**: System MUST generate a static website with exactly 6 chapters: (1) Introduction to Physical AI, (2) Basics of Humanoid Robotics, (3) ROS 2 Fundamentals, (4) Digital Twin Simulation, (5) Vision-Language-Action Systems, (6) Capstone
- **FR-002**: Each chapter MUST be written in markdown format with headings (H1-H3), code blocks, and inline formatting
- **FR-003**: Each chapter MUST have an estimated reading time of 15-30 minutes
- **FR-004**: System MUST generate sidebar navigation automatically from markdown file structure and headings
- **FR-005**: Sidebar MUST display chapters in sequential order and support nested sections based on heading hierarchy

#### RAG Chatbot

- **FR-006**: System MUST provide a chatbot interface accessible from any page
- **FR-007**: Chatbot MUST only answer questions using content from the 6 textbook chapters (no external knowledge)
- **FR-008**: Chatbot responses MUST include citations referencing specific chapter sections
- **FR-009**: When user asks out-of-scope questions, chatbot MUST respond with "I can only answer questions based on the textbook content"
- **FR-010**: System MUST generate embeddings from markdown chapter content using lightweight CPU-based model (no GPU required)
- **FR-011**: Embeddings MUST be stored in vector database (Qdrant Cloud free tier)
- **FR-012**: Metadata about embeddings (chapter, section, page) MUST be stored in relational database (Neon free tier)
- **FR-013**: RAG backend MUST retrieve top 3-5 most relevant text chunks for each query
- **FR-014**: System MUST respond to queries within 2 seconds end-to-end

#### Text Selection Feature

- **FR-015**: Users MUST be able to select any text on a chapter page
- **FR-016**: When text is selected, system MUST display an "Ask AI" button or trigger
- **FR-017**: Clicking "Ask AI" MUST pre-populate the chatbot with a question about the selected text
- **FR-018**: System MUST handle text selections up to 200 words (truncate longer selections)

#### Deployment and Performance

- **FR-019**: System MUST build successfully using Docusaurus static site generator
- **FR-020**: Build process MUST complete in under 2 minutes
- **FR-021**: System MUST deploy to GitHub Pages via automated workflow
- **FR-022**: All pages MUST load within 3 seconds on 3G connection
- **FR-023**: System MUST be responsive and functional on mobile, tablet, and desktop screens
- **FR-024**: All internal links MUST resolve correctly (no 404 errors)

#### Optional Features

- **FR-025**: System MAY support Urdu language translation for all chapter content (if implemented, must maintain parallel markdown files)
- **FR-026**: System MAY include a personalized welcome chapter as first page (if implemented, must be dynamically generated or customizable)

### Key Entities

- **Chapter**: A complete learning module covering one major topic. Contains title, slug, order number, estimated reading time, markdown content, and sections
- **Section**: A subsection within a chapter, derived from H2/H3 headings. Contains title, slug, parent chapter, and text content
- **Embedding**: Vector representation of a text chunk. Contains vector array, source chapter, source section, chunk text, and metadata
- **Query**: User question submitted to RAG chatbot. Contains question text, timestamp, and retrieved chunks
- **Response**: AI-generated answer to a query. Contains answer text, citations (chapter/section references), and confidence score

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can read all 6 chapters sequentially, with each chapter taking 15-30 minutes, totaling 90-180 minutes for complete textbook
- **SC-002**: Build process completes successfully with exit code 0 and generates deployable static site in under 2 minutes
- **SC-003**: Deployed site is accessible via GitHub Pages URL and loads homepage within 3 seconds on standard broadband connection
- **SC-004**: 95% of user queries to RAG chatbot receive relevant answers sourced from textbook chapters within 2 seconds
- **SC-005**: Chatbot correctly refuses to answer out-of-scope questions (non-textbook topics) 100% of the time in testing
- **SC-006**: All internal chapter links and navigation items resolve correctly (0 broken links)
- **SC-007**: Text selection feature works on 100% of paragraphs and code blocks without JavaScript errors
- **SC-008**: Site is responsive and readable on devices with screen widths from 320px (mobile) to 1920px (desktop)
- **SC-009**: RAG system operates within free-tier limits: <1GB Qdrant storage, <0.5GB Neon storage, no paid API calls
- **SC-010**: Users can locate any topic using sidebar navigation in under 10 seconds
- **SC-011**: Code examples in all chapters are syntactically correct and runnable (validated via automated testing)
- **SC-012**: 90% of first-time users successfully interact with chatbot (ask at least one question) within their first session

### Quality Gates

- All markdown lints without errors
- Docusaurus build exits with code 0
- RAG retrieval accuracy >90% on test query set (20+ sample questions)
- Accessibility audit passes WCAG AA standards
- No console errors on any page in target browsers (Chrome, Firefox, Safari)

## Assumptions

1. **Content Authoring**: Chapter markdown files will be authored separately (out of scope for this specification)
2. **Hosting**: GitHub Pages supports static site hosting for free and allows custom domains
3. **Free Tier Capacity**: Qdrant 1GB and Neon 0.5GB are sufficient for 6 chapters of content (estimated <500MB total)
4. **Embedding Model**: sentence-transformers model `all-MiniLM-L6-v2` provides sufficient accuracy for educational Q&A
5. **User Load**: Free-tier rate limits are sufficient for educational use case (estimated <1000 queries/day)
6. **RAG Backend Hosting**: FastAPI backend can be deployed to free-tier hosting (Render, Railway, or similar)
7. **Browser Support**: Target modern browsers (Chrome, Firefox, Safari, Edge) released within last 2 years
8. **No Authentication**: Textbook is public and read-only; no user accounts or login required

## Dependencies

- Docusaurus CLI and Node.js runtime
- GitHub account with GitHub Pages enabled
- Qdrant Cloud free-tier account
- Neon free-tier account
- Python 3.11+ for RAG backend
- Free-tier hosting for FastAPI backend (Render, Railway, or equivalent)

## Out of Scope

- User authentication or personalized learning paths
- Progress tracking or completion certificates
- Downloadable PDF versions of textbook
- Video content or interactive simulations beyond static text and code
- Multi-language support beyond optional Urdu (no French, Spanish, etc.)
- Advanced RAG features (multi-turn conversation memory, fine-tuned models)
- Backend admin panel for content management
- Analytics or user behavior tracking (beyond basic page views)
