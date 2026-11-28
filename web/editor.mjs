// The level editor's page side: toolbar, property fields, mouse and keys. The window itself is drawn by the game (`editor_ui.rs`).
const TOOLS = ['Select', 'Shape', 'Polygon', 'Art', 'Joint', 'Special', 'Text', 'Trigger'];
const INFO = [['character', 'Character'], ['forceCharacter', 'Force character'], ['hideVehicle', 'Hide vehicle'], ['backdrop', 'Backdrop (0-2)'], ['background', 'Background colour']];

export class EditorPanel {
  // `hooks`: `opened()`, `closed()`, `testing(on)`, `status(text)`, `character()`
  constructor(game, specials, hooks) {
    this.game = game;
    this.hooks = hooks;
    this.active = false;
    this.testing = false;
    this.down = false;
    this.space = false;
    this.last = null;
    this.canvas = document.querySelector('#game');
    this.panel = document.querySelector('#editor');
    this.tools = this.panel.querySelector('.tools');
    this.shape = this.panel.querySelector('#ed-shape');
    this.special = this.panel.querySelector('#ed-special');
    this.sliding = this.panel.querySelector('#ed-sliding');
    this.properties = this.panel.querySelector('#ed-properties');
    this.info = this.panel.querySelector('#ed-info');
    TOOLS.forEach((name, i) => {
      const b = document.createElement('button');
      b.type = 'button';
      b.textContent = `${i + 1} ${name}`;
      b.addEventListener('click', () => this.chooseTool(i));
      this.tools.append(b);
    });
    specials.forEach((name, i) => this.special.add(new Option(name, String(i))));
    for (const control of [this.shape, this.special, this.sliding]) control.addEventListener('change', () => this.chooseTool(this.tool));
    this.panel.querySelectorAll('[data-command]').forEach(b => b.addEventListener('click', () => this.command(b.dataset.command)));
    this.panel.querySelector('#ed-file').addEventListener('change', async event => {
      const file = event.target.files[0];
      if (!file) return;
      try {
        game.editor_load(await file.text());
        hooks.status(`Opened ${file.name} in the editor.`);
      } catch (error) {
        hooks.status(`Could not open: ${error}`);
      }
      event.target.value = '';
      this.refresh();
    });
    document.querySelector('#editor-toggle').addEventListener('click', () => (this.active ? this.close() : this.open()));

    const position = event => {
      const r = this.canvas.getBoundingClientRect();
      return [(event.clientX - r.left) * this.canvas.width / r.width, (event.clientY - r.top) * this.canvas.height / r.height];
    };
    this.canvas.addEventListener('pointerdown', event => {
      if (!this.editing()) return;
      event.preventDefault();
      this.canvas.focus();
      this.canvas.setPointerCapture(event.pointerId);
      this.last = position(event);
      // middle button or space + drag moves the view
      this.panning = event.button === 1 || this.space;
      this.down = !this.panning && event.button === 0;
      if (this.down) game.editor_pointer(0, ...this.last, event.shiftKey);
    });
    this.canvas.addEventListener('pointermove', event => {
      if (!this.editing() || !this.last) return;
      const p = position(event);
      game.editor_modifiers(event.shiftKey, event.altKey);
      if (this.panning) game.editor_pan(this.last[0] - p[0], this.last[1] - p[1]);
      else if (this.down) game.editor_pointer(1, ...p, event.shiftKey);
      this.last = p;
    });
    this.canvas.addEventListener('pointerup', event => {
      if (!this.editing()) return;
      if (this.down) game.editor_pointer(2, ...position(event), event.shiftKey);
      this.down = false;
      this.panning = false;
      this.last = null;
      this.refresh();
    });
    this.canvas.addEventListener('dblclick', event => {
      if (!this.editing()) return;
      game.editor_double_click(...position(event));
      this.refresh();
    });
    this.canvas.addEventListener('wheel', event => {
      if (!this.editing()) return;
      event.preventDefault();
      if (event.ctrlKey || event.metaKey) game.editor_zoom(Math.exp(-event.deltaY / 200), ...position(event));
      else game.editor_pan(event.deltaX * 2, event.deltaY * 2);
    }, { passive: false });
  }

  editing() { return this.active && !this.testing; }

  open() {
    this.game.editor_open();
    this.active = true;
    this.testing = false;
    this.panel.hidden = false;
    document.querySelector('#editor-toggle').textContent = 'Close editor';
    this.hooks.opened();
    this.chooseTool(0);
    this.hooks.status('Level editor: keys 1–8 choose a tool, T tests the level, space + drag moves the view.');
  }

  close() {
    this.game.editor_close();
    this.active = false;
    this.testing = false;
    this.panel.hidden = true;
    document.querySelector('#editor-toggle').textContent = 'Level editor';
    this.hooks.closed();
  }

  toggleTest() {
    try {
      if (this.testing) {
        this.game.editor_open();
        this.testing = false;
        this.hooks.status('Back in the editor.');
      } else {
        this.game.editor_test(this.hooks.character());
        this.testing = true;
        this.hooks.status('Testing the level; press T to return to the editor.');
      }
      this.panel.hidden = this.testing;
      this.hooks.testing(this.testing);
    } catch (error) {
      this.hooks.status(`Could not test the level: ${error}`);
    }
  }

  chooseTool(i) {
    this.tool = i;
    this.game.editor_tool(i, Number(this.shape.value), Number(this.special.value), this.sliding.checked);
    this.refresh();
  }

  command(name) {
    if (name === 'test') return this.toggleTest();
    if (name === 'save') {
      const url = URL.createObjectURL(new Blob([this.game.editor_xml()], { type: 'text/xml' }));
      const a = document.createElement('a');
      a.href = url;
      a.download = 'level.xml';
      a.click();
      URL.revokeObjectURL(url);
      return;
    }
    if (name === 'open') return this.panel.querySelector('#ed-file').click();
    if (name === 'new' && !window.confirm('Discard the level in the editor?')) return;
    this.game.editor_command(name);
    this.refresh();
  }

  // Returns true when the event was the editor's.
  key(event, down) {
    if (!this.active) return false;
    const field = event.target instanceof HTMLElement && event.target.closest('input:not([type=checkbox]), select, textarea');
    if (field) return true;
    if (event.keyCode === 84) {
      if (down && !event.repeat) this.toggleTest();
      event.preventDefault();
      return true;
    }
    if (this.testing) return false;
    if (event.code === 'Space') {
      this.space = down;
      event.preventDefault();
      return true;
    }
    if (down && this.game.editor_key(event.keyCode, event.shiftKey, event.ctrlKey || event.metaKey)) {
      event.preventDefault();
      this.refresh();
    }
    return true;
  }

  // the selected trigger's targets and what it does to each
  showTargets(targets) {
    const change = (...args) => { this.game.editor_trigger(...args); this.refresh(); };
    const button = (text, apply) => {
      const b = document.createElement('button');
      b.type = 'button';
      b.textContent = text;
      b.addEventListener('click', apply);
      return b;
    };
    const row = (...children) => {
      const div = document.createElement('div');
      div.className = 'row';
      div.append(...children);
      return div;
    };
    this.panel.querySelector('#ed-targets').replaceChildren(...targets.map((target, t) => {
      const box = document.createElement('fieldset');
      const legend = document.createElement('legend');
      legend.textContent = target.label;
      box.append(legend);
      target.actions.forEach(([index, params], a) => {
        const choice = document.createElement('select');
        choice.setAttribute('aria-label', `${target.label} action ${a + 1}`);
        target.choices.forEach((name, i) => choice.add(new Option(name, String(i), i === index, i === index)));
        choice.addEventListener('change', () => change(0, t, a, 0, Number(choice.value)));
        const inputs = params.map(([name, value], p) => {
          const label = document.createElement('label');
          const input = document.createElement('input');
          input.type = 'number';
          input.step = 'any';
          input.value = String(value);
          input.addEventListener('change', () => change(3, t, a, p, input.valueAsNumber));
          label.append(`${name} `, input);
          return label;
        });
        const remove = target.actions.length > 1 ? [button('Remove action', () => change(2, t, a, 0, 0))] : [];
        box.append(row(choice, ...inputs, ...remove));
      });
      const add = target.actions.length && target.actions.length < 10 ? [button('Add action', () => change(1, t, 0, 0, 0))] : [];
      box.append(row(...add, button('Remove target', () => change(4, t, 0, 0, 0))));
      return box;
    }));
  }

  refresh() {
    if (!this.editing()) return;
    const state = JSON.parse(this.game.editor_state());
    this.tool = state.tool;
    [...this.tools.children].forEach((b, i) => b.setAttribute('aria-pressed', String(i === state.tool)));
    const field = (name, value, apply) => {
      const label = document.createElement('label');
      const input = document.createElement('input');
      if (typeof value === 'boolean') {
        input.type = 'checkbox';
        input.checked = value;
        input.addEventListener('change', () => apply(String(input.checked)));
      } else {
        input.type = 'text';
        input.value = String(value);
        input.size = name === 'caption' ? 30 : 8;
        input.addEventListener('change', () => apply(input.value));
      }
      label.append(`${name} `, input);
      return label;
    };
    const set = (name, value) => { this.game.editor_set(name, value); this.refresh(); };
    this.panel.querySelector('#ed-selected').textContent = state.editingVertices !== null
      ? `Editing vertices (${state.editingVertices} selected): drag to move, Enter inserts, Backspace removes, double-click empty space to finish.`
      : state.selected ? `${state.selected} selected` : 'Nothing selected';
    this.properties.replaceChildren(...state.properties.map(([name, value]) => field(name, value, v => set(name, v))));
    this.showTargets(state.targets);
    this.info.replaceChildren(...INFO.map(([key, label]) => {
      const row = field(label, state.info[key], v => { this.game.editor_set_info(key, v); this.refresh(); });
      return row;
    }));
  }
}
