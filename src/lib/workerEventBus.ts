type Listener<Payload> = (payload: Payload) => void;

export function createPubSub<Events extends Record<string, unknown>>() {
  const listeners = new Map<keyof Events, Set<Listener<Events[keyof Events]>>>();

  return {
    publish<Key extends keyof Events>(type: Key, payload: Events[Key]) {
      const typedListeners = listeners.get(type) as Set<Listener<Events[Key]>> | undefined;
      typedListeners?.forEach((listener) => {
        listener(payload);
      });
    },

    subscribe<Key extends keyof Events>(type: Key, listener: Listener<Events[Key]>) {
      let typedListeners = listeners.get(type) as Set<Listener<Events[Key]>> | undefined;
      if (!typedListeners) {
        typedListeners = new Set<Listener<Events[Key]>>();
        listeners.set(type, typedListeners as Set<Listener<Events[keyof Events]>>);
      }
      typedListeners.add(listener);
      return () => {
        typedListeners.delete(listener);
        if (typedListeners.size === 0) {
          listeners.delete(type);
        }
      };
    },

    clear() {
      listeners.forEach((listenerSet) => listenerSet.clear());
      listeners.clear();
    },
  };
}
