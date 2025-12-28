import os
import re
from pathlib import Path
from sklearn.feature_extraction.text import TfidfVectorizer
from sklearn.metrics.pairwise import cosine_similarity
import numpy as np

class SimpleRAG:
    def __init__(self, docs_path="../docs"):
        """Initialize simple RAG with TF-IDF"""
        self.docs_path = Path(docs_path)
        self.documents = []
        self.metadatas = []
        self.vectorizer = TfidfVectorizer(
            max_features=1000,
            stop_words='english',
            ngram_range=(1, 2)
        )
        self._load_book_content()
    
    def _load_book_content(self):
        """Load all markdown files"""
        print("Loading book content...")
        
        md_files = list(self.docs_path.rglob("*.md"))
        
        for md_file in md_files:
            with open(md_file, 'r', encoding='utf-8') as f:
                content = f.read()
            
            sections = self._split_by_sections(content, md_file.stem)
            
            for heading, text in sections:
                if text.strip() and len(text) > 50:
                    self.documents.append(text)
                    self.metadatas.append({
                        "source": md_file.stem,
                        "heading": heading
                    })
        
        if self.documents:
            self.doc_vectors = self.vectorizer.fit_transform(self.documents)
            print(f"[OK] Loaded {len(self.documents)} sections")
        else:
            print("[WARN] No documents loaded")
    
    def _split_by_sections(self, content, chapter_name):
        """Split markdown by headings"""
        lines = content.split('\n')
        sections = []
        current_heading = chapter_name
        current_text = []
        
        for line in lines:
            # Skip YAML frontmatter
            if line.strip() == '---':
                continue
            
            if line.startswith('## '):
                if current_text:
                    sections.append((current_heading, '\n'.join(current_text)))
                current_heading = line.replace('## ', '').strip()
                current_text = []
            elif line.startswith('# '):
                current_heading = line.replace('# ', '').strip()
            else:
                current_text.append(line)
        
        if current_text:
            sections.append((current_heading, '\n'.join(current_text)))
        
        return sections
    
    def query(self, question, n_results=3):
        """Find most relevant sections"""
        if not self.documents:
            return "", []
        
        # Vectorize query
        query_vec = self.vectorizer.transform([question])
        
        # Calculate similarities
        similarities = cosine_similarity(query_vec, self.doc_vectors)[0]
        
        # Get top results
        top_indices = np.argsort(similarities)[-n_results:][::-1]
        
        context_parts = []
        sources = []
        
        for idx in top_indices:
            if similarities[idx] > 0.1:  # Threshold
                metadata = self.metadatas[idx]
                doc = self.documents[idx]
                
                # Clean and truncate
                clean_doc = doc.strip()[:500]
                
                context_parts.append(f"[{metadata['heading']}]\n{clean_doc}")
                sources.append({
                    'source': metadata['source'],
                    'heading': metadata['heading'],
                    'relevance': float(similarities[idx])
                })
        
        context = "\n\n---\n\n".join(context_parts)
        return context, sources

# Global instance
_rag = None

def get_rag():
    """Get or create RAG singleton"""
    global _rag
    if _rag is None:
        possible_paths = [
            Path(__file__).parent.parent.parent.parent / "docs",
            Path(__file__).parent.parent.parent / "docs",
            Path("../../docs"),
            Path("docs")
        ]
        
        docs_path = None
        for path in possible_paths:
            if path.exists() and path.is_dir():
                docs_path = path
                print(f"Found docs at: {docs_path}")
                break
        
        if docs_path is None:
            raise FileNotFoundError("Could not find docs directory")
        
        _rag = SimpleRAG(docs_path)
    
    return _rag
