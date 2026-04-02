import { useEffect } from 'react';

import type { Mode } from '../lib/appModel';

type UseKeyboardShortcutsParams = {
  setMode: (mode: Mode) => void;
  handleBrake: () => Promise<void>;
  handleCancel: () => Promise<void>;
  handleRestart: () => Promise<void>;
};

export function useKeyboardShortcuts({
  setMode,
  handleBrake,
  handleCancel,
  handleRestart,
}: UseKeyboardShortcutsParams) {
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.repeat) {
        return;
      }

      switch (event.key.toLowerCase()) {
        case 'a':
          event.preventDefault();
          setMode('goal');
          break;
        case 's':
          event.preventDefault();
          setMode('pose');
          break;
        case 'd':
          event.preventDefault();
          void handleBrake();
          break;
        case 'f':
          event.preventDefault();
          void handleCancel();
          break;
        case 'r':
          event.preventDefault();
          void handleRestart();
          break;
        default:
          break;
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [handleBrake, handleCancel, handleRestart, setMode]);
}
