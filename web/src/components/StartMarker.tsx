import { useMemo } from 'react';
import type { Vec3 } from '../types';

function toThreePos([x, y, z]: Vec3): [number, number, number] {
  return [x, z, y];
}

interface StartMarkerProps {
  position: Vec3;
  heading: number;
}

export function StartMarker({ position, heading }: StartMarkerProps) {
  const pos = useMemo(() => toThreePos(position), [position]);
  const arrowRot = useMemo(
    () => [0, -(heading * Math.PI) / 180, 0] as const,
    [heading],
  );

  return (
    <group>
      {/* Green sphere */}
      <mesh position={pos}>
        <sphereGeometry args={[150, 32, 16]} />
        <meshStandardMaterial color="#00ff88" />
      </mesh>
      {/* White heading arrow (cone pointing forward) */}
      <mesh position={pos} rotation={arrowRot}>
        <coneGeometry args={[60, 200, 8]} />
        <meshStandardMaterial color="#ffffff" />
      </mesh>
    </group>
  );
}
