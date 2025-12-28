import './App.css'
import Chatbot from './components/Chatbot'

function App() {
  return (
    <>
      <div className="container">
        <h1>Physical AI & Humanoid Robotics</h1>
        <h2>Interactive Textbook Assistant</h2>

        <div className="card">
          <h3>Welcome to the AI-Powered Textbook</h3>
          <p>Created by: Hooria Arshad</p>
          <p>Ask questions about Physical AI, Humanoid Robotics, ROS 2, and more!</p>

          <div className="links">
            <a href="https://github.com/HooriaArshad" target="_blank" rel="noopener noreferrer">
              GitHub
            </a>
            <a href="https://www.linkedin.com/in/hooria-arshad-hoor-ba32b5301/" target="_blank" rel="noopener noreferrer">
              LinkedIn
            </a>
            <a href="https://www.instagram.com/call_me_hoora_/" target="_blank" rel="noopener noreferrer">
              Instagram
            </a>
          </div>
        </div>

        <Chatbot />
      </div>
    </>
  )
}

export default App
