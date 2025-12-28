import { useState } from 'react'
import './App.css'

function App() {
  const [count, setCount] = useState(0)

  return (
    <>
      <div className="container">
        <h1>Physical AI & Humanoid Robotics</h1>
        <h2>Frontend Dashboard</h2>
        
        <div className="card">
          <h3>Welcome to Docasuros Frontend</h3>
          <p>Created by: Hooria Arshad</p>
          
          <div className="counter">
            <button onClick={() => setCount((count) => count + 1)}>
              Count is {count}
            </button>
          </div>
          
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
      </div>
    </>
  )
}

export default App
