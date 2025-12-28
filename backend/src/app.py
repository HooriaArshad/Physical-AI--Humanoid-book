from flask import Flask, request, jsonify
from flask_cors import CORS
import os
from openai import OpenAI

app = Flask(__name__)
CORS(app)

# Initialize OpenAI client
client = OpenAI(api_key=os.getenv('OPENAI_API_KEY', 'your-api-key-here'))

# Book content knowledge base
BOOK_CONTEXT = """
You are an AI assistant for the "Physical AI & Humanoid Robotics — Essentials" textbook by Hooria Arshad.

BOOK STRUCTURE:
- Preface: Introduction to Physical AI convergence
- Chapter 1: Introduction to Physical AI (perception, reasoning, actuation, VLA models)
- Chapter 2: Basics of Humanoid Robotics (DoF, actuation, locomotion, manipulation)
- Chapter 3: ROS 2 Fundamentals (nodes, topics, services, actions)
- Chapter 4: Digital Twin Simulation (Gazebo, NVIDIA Isaac Sim, sim-to-real)
- Chapter 5: Vision-Language-Action Systems (RT-1, RT-2, PaLM-E)
- Chapter 6: Capstone Project (end-to-end AI-robot pipeline)

KEY CONCEPTS:
- Physical AI: AI systems with embodied agents interacting with physical world
- Humanoid Robots: Atlas, Tesla Optimus, Figure 01, Digit
- ROS 2: Middleware for robot communication
- Digital Twin: Virtual replica for testing
- VLA Models: Vision + Language + Action integration
- Sim-to-Real Gap: Challenges transferring from simulation to reality

Answer questions about the textbook content, help with understanding concepts, and provide code examples when relevant.
"""

@app.route('/api/chat', methods=['POST'])
def chat():
    try:
        data = request.json
        user_message = data.get('message', '')
        
        if not user_message:
            return jsonify({'error': 'No message provided'}), 400
        
        # Create chat completion with book context
        response = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": BOOK_CONTEXT},
                {"role": "user", "content": user_message}
            ],
            max_tokens=500,
            temperature=0.7
        )
        
        assistant_message = response.choices[0].message.content
        
        return jsonify({
            'response': assistant_message,
            'success': True
        })
        
    except Exception as e:
        return jsonify({
            'error': str(e),
            'success': False
        }), 500

@app.route('/api/health', methods=['GET'])
def health():
    return jsonify({'status': 'healthy'})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
