from flask import Flask, request, jsonify
from flask_cors import CORS
import os
from openai import OpenAI
from utils.simple_rag import get_rag

app = Flask(__name__)
CORS(app)

# Initialize OpenAI client
try:
    api_key = os.getenv('OPENAI_API_KEY', '')
    if api_key and api_key != 'your-api-key-here':
        client = OpenAI(api_key=api_key)
        USE_OPENAI = True
        print("[OK] OpenAI client initialized")
    else:
        client = None
        USE_OPENAI = False
        print("[WARN] No OpenAI API key - using fallback mode")
except Exception as e:
    client = None
    USE_OPENAI = False
    print(f"[WARN] OpenAI init failed: {e}")

# Initialize RAG
try:
    rag = get_rag()
    print("[OK] RAG system ready")
except Exception as e:
    print(f"[WARN] RAG init failed: {e}")
    rag = None

SYSTEM_PROMPT = """You are an AI assistant for "Physical AI & Humanoid Robotics — Essentials" by Hooria Arshad.

Answer questions based ONLY on the provided context from the book. Be concise and accurate."""

def generate_fallback_response(context, question):
    """Generate response without OpenAI API"""
    if not context:
        return "I'm sorry, I couldn't find relevant information in the textbook to answer your question. Please try asking about topics like Physical AI, Humanoid Robotics, ROS 2, Digital Twin Simulation, or Vision-Language-Action Systems."
    
    # Simple extraction of most relevant paragraph
    paragraphs = context.split('\n\n')
    relevant = paragraphs[0] if paragraphs else context[:500]
    
    return f"Based on the textbook:\n\n{relevant}\n\n---\n\n💡 This is an extracted response from the book. For better AI-generated answers, please configure an OpenAI API key."

@app.route('/api/chat', methods=['POST'])
def chat():
    try:
        data = request.json
        user_message = data.get('message', '')
        
        if not user_message:
            return jsonify({'error': 'No message provided'}), 400
        
        # Get relevant context from RAG
        context = ""
        sources = []
        
        if rag:
            context, sources = rag.query(user_message, n_results=3)
        
        # Try OpenAI first, fallback to simple response
        if USE_OPENAI and client:
            try:
                prompt = f"""Context from textbook:

{context}

---

Question: {user_message}

Answer:"""
                
                response = client.chat.completions.create(
                    model="gpt-3.5-turbo",
                    messages=[
                        {"role": "system", "content": SYSTEM_PROMPT},
                        {"role": "user", "content": prompt}
                    ],
                    max_tokens=500,
                    temperature=0.7
                )
                
                assistant_message = response.choices[0].message.content
                
            except Exception as e:
                # OpenAI failed, use fallback
                print(f"[WARN] OpenAI error: {e}")
                assistant_message = generate_fallback_response(context, user_message)
        else:
            # No OpenAI, use fallback
            assistant_message = generate_fallback_response(context, user_message)
        
        return jsonify({
            'response': assistant_message,
            'success': True,
            'sources': sources,
            'using_openai': USE_OPENAI
        })
        
    except Exception as e:
        return jsonify({
            'error': str(e),
            'success': False
        }), 500

@app.route('/api/health', methods=['GET'])
def health():
    return jsonify({
        'status': 'healthy',
        'rag_enabled': rag is not None,
        'openai_enabled': USE_OPENAI
    })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
