import { useCallback } from 'react';
import type {
  InputConfig,
  PlanResult,
  RadarThreat,
  NoFlyZone,
} from '../types';

interface ControlPanelProps {
  config: InputConfig;
  onConfigChange: (config: InputConfig) => void;
  onPlan: () => void;
  result: PlanResult | null;
  loading: boolean;
  activeClickMode: 'start' | 'target' | null;
  onSetClickMode: (mode: 'start' | 'target' | null) => void;
}

export function ControlPanel({
  config,
  onConfigChange,
  onPlan,
  result,
  loading,
  activeClickMode,
  onSetClickMode,
}: ControlPanelProps) {
  const update = (patch: Partial<InputConfig>) =>
    onConfigChange({ ...config, ...patch });

  const updateRoute = (patch: Partial<InputConfig['route_definition']>) =>
    update({ route_definition: { ...config.route_definition, ...patch } });

  const updateStart = (
    patch: Partial<InputConfig['route_definition']['start_state']>,
  ) =>
    updateRoute({
      start_state: {
        ...config.route_definition.start_state,
        ...patch,
      },
    });

  const updateTarget = (
    patch: Partial<InputConfig['route_definition']['target']>,
  ) =>
    updateRoute({
      target: { ...config.route_definition.target, ...patch },
    });

  const updateVehicle = (patch: Partial<InputConfig['vehicle']>) =>
    update({ vehicle: { ...config.vehicle, ...patch } });

  const updateSession = (patch: Partial<InputConfig['session']>) =>
    update({ session: { ...config.session, ...patch } });

  const addRadar = useCallback(() => {
    const id = `radar_${Date.now()}`;
    update({
      environment: {
        ...config.environment,
        radars: [
          ...config.environment.radars,
          { id, center: [5000, 5000, 0], radius: 3000 },
        ],
      },
    });
  }, [config]);

  const addNFZ = useCallback(() => {
    const id = `nfz_${Date.now()}`;
    update({
      environment: {
        ...config.environment,
        no_fly_zones: [
          ...config.environment.no_fly_zones,
          {
            id,
            boundary_points: [
              [3000, 3000],
              [6000, 3000],
              [6000, 6000],
              [3000, 6000],
            ],
            alt_min: 0,
            alt_max: 5000,
          },
        ],
      },
    });
  }, [config]);

  const updateRadar = (id: string, patch: Partial<RadarThreat>) => {
    update({
      environment: {
        ...config.environment,
        radars: config.environment.radars.map((r) =>
          r.id === id ? { ...r, ...patch } : r,
        ),
      },
    });
  };

  const removeRadar = (id: string) => {
    update({
      environment: {
        ...config.environment,
        radars: config.environment.radars.filter((r) => r.id !== id),
      },
    });
  };

  const updateNFZ = (id: string, patch: Partial<NoFlyZone>) => {
    update({
      environment: {
        ...config.environment,
        no_fly_zones: config.environment.no_fly_zones.map((n) =>
          n.id === id ? { ...n, ...patch } : n,
        ),
      },
    });
  };

  const removeNFZ = (id: string) => {
    update({
      environment: {
        ...config.environment,
        no_fly_zones: config.environment.no_fly_zones.filter(
          (n) => n.id !== id,
        ),
      },
    });
  };

  return (
    <div>
      <h2>Simple Router Planner</h2>

      {/* Click mode toggle */}
      <h3>场景操作</h3>
      <div className="mode-buttons">
        <button
          className={activeClickMode === 'start' ? 'active' : ''}
          onClick={() =>
            onSetClickMode(activeClickMode === 'start' ? null : 'start')
          }
        >
          起点
        </button>
        <button
          className={activeClickMode === 'target' ? 'active' : ''}
          onClick={() =>
            onSetClickMode(activeClickMode === 'target' ? null : 'target')
          }
        >
          终点
        </button>
      </div>

      {/* Start state */}
      <h3>起点状态</h3>
      <div className="field-row">
        <div>
          <label>X (m)</label>
          <input
            type="number"
            value={config.route_definition.start_state.position[0]}
            onChange={(e) =>
              updateStart({
                position: [
                  +e.target.value,
                  config.route_definition.start_state.position[1],
                  config.route_definition.start_state.position[2],
                ],
              })
            }
          />
        </div>
        <div>
          <label>Y (m)</label>
          <input
            type="number"
            value={config.route_definition.start_state.position[1]}
            onChange={(e) =>
              updateStart({
                position: [
                  config.route_definition.start_state.position[0],
                  +e.target.value,
                  config.route_definition.start_state.position[2],
                ],
              })
            }
          />
        </div>
        <div>
          <label>Z (m)</label>
          <input
            type="number"
            value={config.route_definition.start_state.position[2]}
            onChange={(e) =>
              updateStart({
                position: [
                  config.route_definition.start_state.position[0],
                  config.route_definition.start_state.position[1],
                  +e.target.value,
                ],
              })
            }
          />
        </div>
      </div>
      <div>
        <label>Heading (°)</label>
        <input
          type="number"
          value={config.route_definition.start_state.heading_deg}
          onChange={(e) =>
            updateStart({ heading_deg: +e.target.value })
          }
        />
      </div>

      {/* Target zone */}
      <h3>目标区域</h3>
      <div className="field-row">
        <div>
          <label>X (m)</label>
          <input
            type="number"
            value={config.route_definition.target.center[0]}
            onChange={(e) =>
              updateTarget({
                center: [
                  +e.target.value,
                  config.route_definition.target.center[1],
                  config.route_definition.target.center[2],
                ],
              })
            }
          />
        </div>
        <div>
          <label>Y (m)</label>
          <input
            type="number"
            value={config.route_definition.target.center[1]}
            onChange={(e) =>
              updateTarget({
                center: [
                  config.route_definition.target.center[0],
                  +e.target.value,
                  config.route_definition.target.center[2],
                ],
              })
            }
          />
        </div>
        <div>
          <label>Z (m)</label>
          <input
            type="number"
            value={config.route_definition.target.center[2]}
            onChange={(e) =>
              updateTarget({
                center: [
                  config.route_definition.target.center[0],
                  config.route_definition.target.center[1],
                  +e.target.value,
                ],
              })
            }
          />
        </div>
      </div>
      <div>
        <label>半径 (m)</label>
        <input
          type="number"
          value={config.route_definition.target.radius}
          onChange={(e) => updateTarget({ radius: +e.target.value })}
          min={0}
        />
      </div>

      {/* Vehicle */}
      <h3>飞行器参数</h3>
      <div>
        <label>最小转弯半径 (m)</label>
        <input
          type="number"
          value={config.vehicle.min_turn_radius}
          onChange={(e) =>
            updateVehicle({ min_turn_radius: +e.target.value })
          }
        />
      </div>
      <div>
        <label>最大爬升角 (°)</label>
        <input
          type="number"
          value={config.vehicle.max_climb_angle}
          onChange={(e) =>
            updateVehicle({ max_climb_angle: +e.target.value })
          }
        />
      </div>
      <div>
        <label>最大转弯角 (°)</label>
        <input
          type="number"
          value={config.vehicle.max_turn_angle_deg}
          onChange={(e) =>
            updateVehicle({ max_turn_angle_deg: +e.target.value })
          }
        />
      </div>

      {/* Session */}
      <h3>计算参数</h3>
      <div>
        <label>最大计算时间 (ms)</label>
        <input
          type="number"
          value={config.session.max_calculation_time_ms ?? 5000}
          onChange={(e) =>
            updateSession({ max_calculation_time_ms: +e.target.value })
          }
        />
      </div>
      <div>
        <label>随机种子 (留空自动)</label>
        <input
          type="number"
          value={config.session.seed ?? ''}
          onChange={(e) =>
            updateSession({
              seed: e.target.value ? +e.target.value : null,
            })
          }
          placeholder="auto"
        />
      </div>

      {/* Radars */}
      <h3>雷达威胁 ({config.environment.radars.length})</h3>
      {config.environment.radars.map((r) => (
        <div key={r.id} className="obstacle-item">
          <div className="obstacle-header">
            <span>{r.id}</span>
            <button
              className="btn-small btn-danger"
              onClick={() => removeRadar(r.id)}
            >
              ✕
            </button>
          </div>
          <div className="field-row">
            <div>
              <label>X</label>
              <input
                type="number"
                value={r.center[0]}
                onChange={(e) =>
                  updateRadar(r.id, {
                    center: [+e.target.value, r.center[1], r.center[2]],
                  })
                }
              />
            </div>
            <div>
              <label>Y</label>
              <input
                type="number"
                value={r.center[1]}
                onChange={(e) =>
                  updateRadar(r.id, {
                    center: [r.center[0], +e.target.value, r.center[2]],
                  })
                }
              />
            </div>
            <div>
              <label>Z</label>
              <input
                type="number"
                value={r.center[2]}
                onChange={(e) =>
                  updateRadar(r.id, {
                    center: [r.center[0], r.center[1], +e.target.value],
                  })
                }
              />
            </div>
          </div>
          <div>
            <label>半径</label>
            <input
              type="number"
              value={r.radius}
              onChange={(e) =>
                updateRadar(r.id, { radius: +e.target.value })
              }
            />
          </div>
        </div>
      ))}
      <button
        className="btn-small"
        onClick={addRadar}
        style={{ width: '100%', background: '#333', color: '#e0e0e0' }}
      >
        + 添加雷达
      </button>

      {/* NFZs */}
      <h3>禁飞区 ({config.environment.no_fly_zones.length})</h3>
      {config.environment.no_fly_zones.map((n) => (
        <div key={n.id} className="obstacle-item">
          <div className="obstacle-header">
            <span>{n.id}</span>
            <button
              className="btn-small btn-danger"
              onClick={() => removeNFZ(n.id)}
            >
              ✕
            </button>
          </div>
          <div>
            <label>最低高度</label>
            <input
              type="number"
              value={n.alt_min}
              onChange={(e) =>
                updateNFZ(n.id, { alt_min: +e.target.value })
              }
            />
          </div>
          <div>
            <label>最高高度</label>
            <input
              type="number"
              value={n.alt_max}
              onChange={(e) =>
                updateNFZ(n.id, { alt_max: +e.target.value })
              }
            />
          </div>
          <div>
            <label>边界点 ({n.boundary_points.length} 个)</label>
            {n.boundary_points.map((pt, i) => (
              <div key={i} className="field-row" style={{ marginTop: 4 }}>
                <input
                  type="number"
                  value={pt[0]}
                  onChange={(e) => {
                    const pts = [...n.boundary_points];
                    pts[i] = [+e.target.value, pts[i][1]];
                    updateNFZ(n.id, { boundary_points: pts });
                  }}
                  placeholder="x"
                />
                <input
                  type="number"
                  value={pt[1]}
                  onChange={(e) => {
                    const pts = [...n.boundary_points];
                    pts[i] = [pts[i][0], +e.target.value];
                    updateNFZ(n.id, { boundary_points: pts });
                  }}
                  placeholder="y"
                />
                <button
                  className="btn-small btn-danger"
                  onClick={() =>
                    updateNFZ(n.id, {
                      boundary_points: n.boundary_points.filter(
                        (_, j) => j !== i,
                      ),
                    })
                  }
                >
                  ✕
                </button>
              </div>
            ))}
            <button
              className="btn-small"
              style={{
                marginTop: 4,
                background: '#333',
                color: '#e0e0e0',
              }}
              onClick={() =>
                updateNFZ(n.id, {
                  boundary_points: [
                    ...n.boundary_points,
                    [5000, 5000],
                  ],
                })
              }
            >
              + 添加顶点
            </button>
          </div>
        </div>
      ))}
      <button
        className="btn-small"
        onClick={addNFZ}
        style={{ width: '100%', background: '#333', color: '#e0e0e0' }}
      >
        + 添加禁飞区
      </button>

      {/* Plan button */}
      <button
        onClick={onPlan}
        disabled={loading}
        style={{
          marginTop: 16,
          padding: '14px',
          fontSize: 16,
          width: '100%',
        }}
      >
        {loading ? '⏳ 计算中...' : '开始规划'}
      </button>

      {/* Results */}
      {result && result.status === 'SUCCESS' && (
        <div className="result">
          <strong>✅ 规划成功</strong>
          <div>
            路径长度: {(result.summary.total_length_m / 1000).toFixed(2)} km
          </div>
          <div>
            最大爬升角: {result.summary.max_climb_angle_utilized.toFixed(1)}°
          </div>
          <div>节点探索: {result.diagnostics.nodes_explored}</div>
          <div>
            计算时间: {result.diagnostics.calculation_time_ms.toFixed(0)} ms
          </div>
          <div>路点数量: {result.waypoints.length}</div>
        </div>
      )}

      {result && result.status === 'FAILED' && (
        <div className="result" style={{ color: '#ff6666' }}>
          <strong>❌ 规划失败</strong>
          <div>错误: {result.error.message}</div>
          <div>代码: {result.error.code}</div>
        </div>
      )}
    </div>
  );
}
