import './App.css';
import Panel from './components/Panel';
import React from 'react';


function App() {
  return (
    <React.Fragment>
      <div className="App">
        <header className="App-header">
          Welcome to remrob
        </header>
        <Panel/>
      </div>
    </React.Fragment>
  );
}

export default App;
