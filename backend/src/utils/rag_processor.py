import os
import chromadb
from chromadb.config import Settings
from sentence_transformers import SentenceTransformer
import markdown
from pathlib import Path

class RAGProcessor:
    def __init__(self, docs_path="../docs"):
        """Initialize RAG processor with ChromaDB and embeddings model"""
        self.docs_path = Path(docs_path)
        self.embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
        
        # Initialize ChromaDB
        self.client = chromadb.Client(Settings(
            anonymized_telemetry=False,
            is_persistent=False
        ))
        
        # Create or get collection
        self.collection = self.client.get_or_create_collection(
            name="book_content",
            metadata={"description": "Physical AI & Humanoid Robotics textbook content"}
        )
        
        # Load book content if collection is empty
        if self.collection.count() == 0:
            self._load_book_content()
    
    def _load_book_content(self):
        """Load all markdown files from docs directory"""
        print("Loading book content into vector database...")
        
        # Find all markdown files
        md_files = list(self.docs_path.rglob("*.md"))
        
        documents = []
        metadatas = []
        ids = []
        
        for idx, md_file in enumerate(md_files):
            # Read markdown content
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Split by sections (## headings)
            sections = self._split_by_sections(content, md_file.stem)
            
            for section_idx, (heading, text) in enumerate(sections):
                if text.strip():
                    documents.append(text)
                    metadatas.append({
                        "source": str(md_file.relative_to(self.docs_path)),
                        "heading": heading,
                        "chapter": md_file.stem
                    })
                    ids.append(f"{md_file.stem}_{section_idx}")
        
        # Add to ChromaDB
        if documents:
            self.collection.add(
                documents=documents,
                metadatas=metadatas,
                ids=ids
            )
            print(f"✅ Loaded {len(documents)} sections from {len(md_files)} files")
    
    def _split_by_sections(self, content, chapter_name):
        """Split markdown content by ## headings"""
        lines = content.split('\n')
        sections = []
        current_heading = chapter_name
        current_text = []
        
        for line in lines:
            if line.startswith('## '):
                # Save previous section
                if current_text:
                    sections.append((current_heading, '\n'.join(current_text)))
                # Start new section
                current_heading = line.replace('## ', '').strip()
                current_text = []
            elif line.startswith('# '):
                current_heading = line.replace('# ', '').strip()
            else:
                current_text.append(line)
        
        # Add last section
        if current_text:
            sections.append((current_heading, '\n'.join(current_text)))
        
        return sections
    
    def query(self, question, n_results=3):
        """Query the RAG system for relevant content"""
        # Search ChromaDB
        results = self.collection.query(
            query_texts=[question],
            n_results=n_results
        )
        
        # Format results
        context_parts = []
        sources = []
        
        if results['documents'] and results['documents'][0]:
            for idx, doc in enumerate(results['documents'][0]):
                metadata = results['metadatas'][0][idx]
                context_parts.append(f"[{metadata['heading']}]\n{doc}")
                sources.append({
                    'source': metadata['source'],
                    'heading': metadata['heading']
                })
        
        context = "\n\n---\n\n".join(context_parts)
        return context, sources

# Global instance
_rag_processor = None

def get_rag_processor():
    """Get or create RAG processor singleton"""
    global _rag_processor
    if _rag_processor is None:
        # Determine docs path (handle different directory structures)
        possible_paths = [
            Path(__file__).parent.parent.parent.parent / "docs",
            Path(__file__).parent.parent.parent / "docs",
            Path("docs")
        ]
        
        docs_path = None
        for path in possible_paths:
            if path.exists():
                docs_path = path
                break
        
        if docs_path is None:
            raise FileNotFoundError("Could not find docs directory")
        
        _rag_processor = RAGProcessor(docs_path)
    
    return _rag_processor
