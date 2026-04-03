type Listener<Payload> = (payload: Payload) => void;

type ListenerMap<Events extends Record<string, unknown>> = {
  [Key in keyof Events]?: Set<Listener<Events[Key]>>;
};

export function createPubSub<Events extends Record<string, unknown>>() {
  const listeners: ListenerMap<Events> = {};

  return {
    publish<Key extends keyof Events>(type: Key, payload: Events[Key]) {
      listeners[type]?.forEach((listener) => {
        listener(payload);
      });
    },

    subscribe<Key extends keyof Events>(type: Key, listener: Listener<Events[Key]>) {
      const typedListeners = (listeners[type] ??= new Set()) as Set<Listener<Events[Key]>>;
      typedListeners.add(listener);
      return () => {
        typedListeners.delete(listener);
        if (typedListeners.size === 0) {
          delete listeners[type];
        }
      };
    },

    clear() {
      for (const listenerSet of Object.values(listeners)) {
        listenerSet?.clear();
      }
    },
  };
}
