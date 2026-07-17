import { useMemo } from 'react';
import type { Vec3 } from '../types';

function toThreePos([x, y, z]: Vec3): [number, number, number] {
  return [x, z, y];
}

interface RadarSphereProps {
  center: Vec3;
  radius: number;
}

export function RadarSphere({ center, radius }: RadarSphereProps) {
  const pos = useMemo(() => toThreePos(center), [center]);

  return (
    <mesh position={pos}>
      <sphereGeometry args={[radius, 48, 24]} />
      <meshBasicMaterial color="#ff4444" transparent opacity={0.2} />
    </mesh>
  );
}
