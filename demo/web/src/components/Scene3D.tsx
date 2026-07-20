import { useCallback } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import type { ThreeEvent } from '@react-three/fiber';
import type { Vec3 } from '../types';
import type { RadarThreat, NoFlyZone } from '../types';
import { StartMarker } from './StartMarker';
import { TargetZone } from './TargetZone';
import { RadarSphere } from './RadarSphere';
import { NFZPrism } from './NFZPrism';
import { PathLine } from './PathLine';

interface Scene3DProps {
  startPosition: Vec3;
  startHeading: number;
  targetCenter: Vec3;
  targetRadius: number;
  radars: RadarThreat[];
  noFlyZones: NoFlyZone[];
  waypoints: Vec3[] | null;
  onGroundClick: (pos: Vec3) => void;
  activeClickMode: 'start' | 'target' | null;
}

function GroundClickPlane({
  active,
  onClick,
}: {
  active: boolean;
  onClick: (pos: Vec3) => void;
}) {
  const { camera, pointer, raycaster } = useThree();

  const handleClick = useCallback(
    (e: ThreeEvent<PointerEvent>) => {
      if (!active) return;
      e.stopPropagation();
      raycaster.setFromCamera(pointer, camera);
      const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
      const intersection = new THREE.Vector3();
      raycaster.ray.intersectPlane(plane, intersection);
      if (intersection) {
        // Three.js [x, y, z] → CLI [x, z, y]
        onClick([intersection.x, intersection.z, intersection.y]);
      }
    },
    [active, onClick, camera, pointer, raycaster],
  );

  return (
    <mesh
      rotation={[-Math.PI / 2, 0, 0]}
      visible={active}
      onClick={handleClick}
    >
      <planeGeometry args={[50000, 50000]} />
      <meshBasicMaterial visible={false} />
    </mesh>
  );
}

export function Scene3D({
  startPosition,
  startHeading,
  targetCenter,
  targetRadius,
  radars,
  noFlyZones,
  waypoints,
  onGroundClick,
  activeClickMode,
}: Scene3DProps) {
  return (
    <Canvas
      camera={{
        position: [15000, 12000, 15000],
        fov: 50,
        near: 10,
        far: 200000,
      }}
      style={{ background: '#0a0a1a' }}
    >
      <ambientLight intensity={0.4} />
      <directionalLight position={[20000, 30000, 10000]} intensity={0.6} />

      <OrbitControls makeDefault maxPolarAngle={Math.PI / 2.1} />

      <Grid
        args={[20000, 20000, 20, 20]}
        position={[10000, 0, 10000]}
        cellSize={1000}
        cellThickness={0.5}
        cellColor="#334455"
        sectionSize={5000}
        sectionThickness={1}
        sectionColor="#556677"
        fadeDistance={80000}
        infiniteGrid
      />

      <StartMarker position={startPosition} heading={startHeading} />
      <TargetZone center={targetCenter} radius={targetRadius} />

      {radars.map((r) => (
        <RadarSphere key={r.id} center={r.center} radius={r.radius} />
      ))}
      {noFlyZones.map((n) => (
        <NFZPrism
          key={n.id}
          boundaryPoints={n.boundary_points}
          altMin={n.alt_min}
          altMax={n.alt_max}
        />
      ))}

      {waypoints && <PathLine waypoints={waypoints} />}

      <GroundClickPlane
        active={activeClickMode !== null}
        onClick={onGroundClick}
      />
    </Canvas>
  );
}
