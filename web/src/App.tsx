import { useState } from 'react';
import type { InputConfig, PlanResult } from './types';
import { defaultInputConfig } from './types';
import { planRoute } from './api';

export default function App() {
  const [config, setConfig] = useState<InputConfig>(defaultInputConfig);
  const [result, setResult] = useState<PlanResult | null>(null);
  const [loading, setLoading] = useState(false);

  const handlePlan = async () => {
    setLoading(true);
    setResult(null);
    try {
      const res = await planRoute(config);
      setResult(res);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="app-layout">
      <div className="panel">
        <h2>AFSIM Planner</h2>
        <button onClick={handlePlan} disabled={loading}>
          {loading ? '计算中...' : '开始规划'}
        </button>
        {result && (
          <pre className="result">{JSON.stringify(result, null, 2)}</pre>
        )}
      </div>
      <div className="canvas" />
    </div>
  );
}
