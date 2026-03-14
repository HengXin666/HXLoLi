/**
 * @aspect/libass-wasm - JavaScript wrapper for libass WASM module
 *
 * Provides a high-level API for rendering ASS/SSA subtitles in the browser,
 * with VSFilterMod extension support (\1img, \2img, \3img, \4img, \fsc).
 */

// Will be replaced by the actual module factory at build time
import createModule from './libass.js';

/**
 * VSFilterMod image channel constants
 */
export const VSFModChannel = {
  PRIMARY: 0,    // \1img - primary fill
  SECONDARY: 1,  // \2img - secondary fill (karaoke)
  BORDER: 2,     // \3img - border/outline
  SHADOW: 3,     // \4img - shadow
};

/**
 * High-level wrapper for the libass WASM renderer.
 */
export class LibassRenderer {
  /** @private */
  constructor() {
    this._module = null;
    this._width = 0;
    this._height = 0;
    this._trackLoaded = false;
    this._initialized = false;
  }

  /**
   * Create and initialize a new renderer.
   * @param {import('./libass').LibassOptions} options
   * @returns {Promise<LibassRenderer>}
   */
  static async create(options) {
    const renderer = new LibassRenderer();
    await renderer._init(options);
    return renderer;
  }

  /**
   * @private
   * @param {import('./libass').LibassOptions} options
   */
  async _init(options) {
    const { canvasWidth, canvasHeight, wasmUrl, defaultFont, defaultFamily } = options;

    // Initialize the Emscripten module
    const moduleConfig = {};
    if (wasmUrl) {
      moduleConfig.locateFile = (path) => {
        if (path.endsWith('.wasm')) return wasmUrl;
        return path;
      };
    }

    this._module = await createModule(moduleConfig);
    this._width = canvasWidth;
    this._height = canvasHeight;

    // Initialize libass
    const ret = this._module._libass_init(canvasWidth, canvasHeight);
    if (ret !== 0) {
      throw new Error(`libass initialization failed (code: ${ret})`);
    }
    this._initialized = true;

    // Set fonts if provided
    if (defaultFont || defaultFamily) {
      this.setFonts(defaultFont || null, defaultFamily || 'sans-serif');
    }

    // Store image loader if provided
    this._imageLoader = options.imageLoader || null;
  }

  /**
   * Destroy the renderer and free resources.
   */
  destroy() {
    if (this._initialized) {
      this._module._libass_destroy();
      this._initialized = false;
    }
    this._module = null;
    this._trackLoaded = false;
  }

  /**
   * Resize the rendering canvas.
   * @param {number} width
   * @param {number} height
   */
  setFrameSize(width, height) {
    this._ensureInit();
    this._width = width;
    this._height = height;
    this._module._libass_set_frame_size(width, height);
  }

  /**
   * Configure font settings.
   * @param {string|null} defaultFont
   * @param {string} defaultFamily
   */
  setFonts(defaultFont, defaultFamily) {
    this._ensureInit();
    const M = this._module;

    const fontPtr = defaultFont ? M.stringToNewUTF8(defaultFont) : 0;
    const familyPtr = M.stringToNewUTF8(defaultFamily || 'sans-serif');

    M._libass_set_fonts(fontPtr, familyPtr);

    if (fontPtr) M._free(fontPtr);
    M._free(familyPtr);
  }

  /**
   * Add a font file.
   * @param {string} name  Font filename
   * @param {Uint8Array} data  Font file data
   */
  addFont(name, data) {
    this._ensureInit();
    const M = this._module;

    // Write font to virtual FS
    const fontPath = `/fonts/${name}`;
    try {
      M.FS.mkdirTree('/fonts');
    } catch (e) { /* ignore if exists */ }
    M.FS.writeFile(fontPath, data);

    // Register with libass
    const namePtr = M.stringToNewUTF8(name);
    const dataPtr = M._malloc(data.length);
    M.HEAPU8.set(data, dataPtr);
    M._libass_add_font(namePtr, dataPtr, data.length);
    M._free(namePtr);
    M._free(dataPtr);
  }

  /**
   * Load a subtitle track.
   * @param {string|Uint8Array} content
   * @returns {import('./libass').TrackInfo}
   */
  loadTrack(content) {
    this._ensureInit();
    const M = this._module;

    let data;
    if (typeof content === 'string') {
      data = new TextEncoder().encode(content);
    } else {
      data = content;
    }

    const dataPtr = M._malloc(data.length + 1);
    M.HEAPU8.set(data, dataPtr);
    M.HEAPU8[dataPtr + data.length] = 0; // null terminator

    const eventCount = M._libass_load_track(dataPtr, data.length);
    M._free(dataPtr);

    if (eventCount < 0) {
      throw new Error('Failed to parse subtitle track');
    }

    this._trackLoaded = true;

    return {
      eventCount,
      playResX: M._libass_get_track_width(),
      playResY: M._libass_get_track_height(),
    };
  }

  /**
   * Free the current subtitle track.
   */
  freeTrack() {
    if (this._initialized && this._trackLoaded) {
      this._module._libass_free_track();
      this._trackLoaded = false;
    }
  }

  /**
   * Render a frame at the given time.
   * @param {number} timeMs  Time in milliseconds
   * @returns {import('./libass').RenderResult}
   */
  renderFrame(timeMs) {
    this._ensureInit();
    if (!this._trackLoaded) {
      throw new Error('No track loaded');
    }

    const M = this._module;
    const changedPtr = M._malloc(4);
    M.setValue(changedPtr, 0, 'i32');

    // Emscripten legalizer 将 i64 参数拆为 (lo: i32, hi: i32)
    // 所以实际签名是 (time_lo, time_hi, changed_ptr)
    const timeMsInt = Math.round(timeMs);
    M._libass_render_frame(timeMsInt, 0, changedPtr);

    const changed = M.getValue(changedPtr, 'i32');
    M._free(changedPtr);

    // Read the render buffer
    const bufPtr = M._libass_get_render_buffer();
    const bufSize = this._width * this._height * 4;
    const buffer = new Uint8ClampedArray(
      M.HEAPU8.buffer, bufPtr, bufSize
    );

    return {
      buffer: new Uint8ClampedArray(buffer), // Copy to avoid dangling ref
      changed,
    };
  }

  /**
   * Set a VSFilterMod texture image for a channel.
   * @param {number} channel  VSFModChannel (0-3)
   * @param {Uint8Array|null} imageData  Image file data or null to clear
   */
  setChannelImage(channel, imageData) {
    this._ensureInit();
    const M = this._module;

    if (!imageData) {
      // Clear channel
      const nullPath = M.stringToNewUTF8('');
      M._vsfmod_set_channel_image(channel, nullPath);
      M._free(nullPath);
      return;
    }

    // Write image to virtual FS
    const imgPath = `/images/channel_${channel}.png`;
    try {
      M.FS.mkdirTree('/images');
    } catch (e) { /* ignore */ }
    M.FS.writeFile(imgPath, imageData);

    // Set the channel image
    const pathPtr = M.stringToNewUTF8(imgPath);
    M._vsfmod_set_channel_image(channel, pathPtr);
    M._free(pathPtr);
  }

  /**
   * Write a file to the Emscripten virtual filesystem.
   * @param {string} path
   * @param {Uint8Array} data
   */
  writeFile(path, data) {
    this._ensureInit();
    const M = this._module;

    // Ensure parent directories exist
    const dir = path.substring(0, path.lastIndexOf('/'));
    if (dir) {
      try {
        M.FS.mkdirTree(dir);
      } catch (e) { /* ignore */ }
    }

    M.FS.writeFile(path, data);
  }

  /**
   * Get the render buffer directly (zero-copy, but may be invalidated).
   * @returns {Uint8ClampedArray}
   */
  getRenderBuffer() {
    this._ensureInit();
    const M = this._module;
    const ptr = M._libass_get_render_buffer();
    const size = this._width * this._height * 4;
    return new Uint8ClampedArray(M.HEAPU8.buffer, ptr, size);
  }

  /** @private */
  _ensureInit() {
    if (!this._initialized) {
      throw new Error('LibassRenderer not initialized. Call LibassRenderer.create() first.');
    }
  }
}

export default LibassRenderer;
