import { useState, useCallback } from 'react';
import { Scene3D } from './components/Scene3D';
import { ControlPanel } from './components/ControlPanel';
import type { InputConfig, PlanResult, Vec3 } from './types';
import { defaultInputConfig } from './types';
import { planRoute } from './api';

export default function App() {
  const [config, setConfig] = useState<InputConfig>(defaultInputConfig);
  const [result, setResult] = useState<PlanResult | null>(null);
  const [loading, setLoading] = useState(false);
  const [clickMode, setClickMode] = useState<'start' | 'target' | null>(null);

  const handlePlan = async () => {
    setLoading(true);
    setResult(null);
    try {
      const res = await planRoute(config);
      setResult(res);
    } catch (err) {
      setResult({
        status: 'FAILED',
        error: {
          code: 'NETWORK_ERROR',
          message: String(err),
          location: [0, 0, 0],
          seed_used: 0,
        },
      });
    } finally {
      setLoading(false);
    }
  };

  const handleGroundClick = useCallback(
    (pos: Vec3) => {
      if (clickMode === 'start') {
        setConfig((prev) => ({
          ...prev,
          route_definition: {
            ...prev.route_definition,
            start_state: {
              ...prev.route_definition.start_state,
              position: pos,
            },
          },
        }));
      } else if (clickMode === 'target') {
        setConfig((prev) => ({
          ...prev,
          route_definition: {
            ...prev.route_definition,
            target: {
              ...prev.route_definition.target,
              center: pos,
            },
          },
        }));
      }
    },
    [clickMode],
  );

  const waypoints: Vec3[] | null =
    result?.status === 'SUCCESS'
      ? result.waypoints.map((w) => w.position)
      : null;

  return (
    <div className="app-layout">
      <div className="panel">
        <ControlPanel
          config={config}
          onConfigChange={setConfig}
          onPlan={handlePlan}
          result={result}
          loading={loading}
          activeClickMode={clickMode}
          onSetClickMode={setClickMode}
        />
      </div>
      <div className="canvas">
        <Scene3D
          startPosition={config.route_definition.start_state.position}
          startHeading={config.route_definition.start_state.heading_deg}
          targetCenter={config.route_definition.target.center}
          targetRadius={config.route_definition.target.radius}
          radars={config.environment.radars}
          noFlyZones={config.environment.no_fly_zones}
          waypoints={waypoints}
          onGroundClick={handleGroundClick}
          activeClickMode={clickMode}
        />
      </div>
    </div>
  );
}
