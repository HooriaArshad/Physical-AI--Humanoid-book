# Physical AI Textbook Chatbot Backend

Flask-based backend for the AI-powered textbook chatbot using OpenAI API.

## Setup

### 1. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 2. Configure Environment Variables

Create a `.env` file:

```bash
cp .env.example .env
```

Edit `.env` and add your OpenAI API key:

```
OPENAI_API_KEY=sk-your-actual-api-key-here
FLASK_ENV=development
```

### 3. Run the Server

```bash
python src/app.py
```

Server will run on `http://localhost:5000`

## API Endpoints

### POST `/api/chat`

Send a message to the chatbot.

**Request:**
```json
{
  "message": "What is Physical AI?"
}
```

**Response:**
```json
{
  "response": "Physical AI refers to artificial intelligence systems...",
  "success": true
}
```

### GET `/api/health`

Check server health.

**Response:**
```json
{
  "status": "healthy"
}
```

## Features

- OpenAI GPT-3.5 integration
- Book-specific knowledge base
- CORS enabled for frontend integration
- Error handling and validation

## Tech Stack

- Flask (Python web framework)
- OpenAI API
- Flask-CORS (Cross-origin resource sharing)
- python-dotenv (Environment management)

---

Created by **Hooria Arshad**
