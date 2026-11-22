const bindings = new Map([
  ['ArrowLeft', 0], ['ArrowRight', 1], ['ArrowUp', 2], ['ArrowDown', 3],
  ['Space', 4], ['KeyZ', 5], ['ShiftLeft', 6], ['ShiftRight', 6],
  ['ControlLeft', 7], ['ControlRight', 7],
]);

export class GameInput {
  constructor(game) { this.game = game; this.held = new Set(); }

  key(code, down, editing) {
    const index = bindings.get(code);
    if (index === undefined || (down && editing)) return false;
    const wasHeld = this.held.has(code);
    if (down) this.held.add(code); else this.held.delete(code);
    if (down || wasHeld) {
      this.game.key(index, [...this.held].some(key => bindings.get(key) === index));
    }
    return down || wasHeld;
  }

  clear() { this.held.clear(); this.game.clear_keys(); }
}
