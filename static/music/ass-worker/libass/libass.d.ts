/**
 * @aspect/libass-wasm - libass compiled to WebAssembly with VSFilterMod extensions
 *
 * Supported VSFilterMod Tags:
 *   \1img(path)  - Primary fill image overlay
 *   \2img(path)  - Secondary fill image overlay
 *   \3img(path)  - Border/outline image overlay
 *   \4img(path)  - Shadow image overlay
 *   \fsc<scale>  - Uniform font scale (shorthand for \fscx + \fscy)
 */

/** Image channel indices for VSFilterMod texture overlays */
export declare const enum VSFModChannel {
  PRIMARY = 0,
  SECONDARY = 1,
  BORDER = 2,
  SHADOW = 3,
}

/** Configuration options for initializing the renderer */
export interface LibassOptions {
  /** Canvas width in pixels */
  canvasWidth: number;
  /** Canvas height in pixels */
  canvasHeight: number;
  /** Path to the .wasm file (defaults to same directory as .js) */
  wasmUrl?: string;
  /** Default font file path (in the virtual filesystem) */
  defaultFont?: string;
  /** Default font family name */
  defaultFamily?: string;
  /**
   * Callback to load image files referenced by \Nimg tags.
   * The function receives the path from the ASS script and should
   * return the image data as a Uint8Array, or null if not found.
   */
  imageLoader?: (path: string) => Promise<Uint8Array | null>;
}

/** Information about a loaded subtitle track */
export interface TrackInfo {
  /** Number of subtitle events (dialogue lines) */
  eventCount: number;
  /** PlayResX from the script header */
  playResX: number;
  /** PlayResY from the script header */
  playResY: number;
}

/** Frame render result */
export interface RenderResult {
  /** RGBA pixel buffer (width * height * 4 bytes) */
  buffer: Uint8ClampedArray;
  /**
   * Whether the frame changed since the last render.
   * 0 = no change, 1 = minor change, 2 = full change
   */
  changed: number;
}

/**
 * Main libass WASM renderer class.
 *
 * Usage:
 * ```typescript
 * const renderer = await LibassRenderer.create({
 *   canvasWidth: 1920,
 *   canvasHeight: 1080,
 * });
 *
 * // Load fonts
 * await renderer.addFont('arial.ttf', fontData);
 * renderer.setFonts(null, 'Arial');
 *
 * // Load subtitle track
 * const track = renderer.loadTrack(assFileContent);
 *
 * // Render a frame at time 5000ms
 * const result = renderer.renderFrame(5000);
 * ctx.putImageData(new ImageData(result.buffer, 1920, 1080), 0, 0);
 *
 * // Clean up
 * renderer.destroy();
 * ```
 */
export declare class LibassRenderer {
  /**
   * Create and initialize a new renderer instance.
   * Downloads and instantiates the WASM module.
   */
  static create(options: LibassOptions): Promise<LibassRenderer>;

  /**
   * Destroy the renderer and free all resources.
   */
  destroy(): void;

  /**
   * Resize the rendering canvas.
   */
  setFrameSize(width: number, height: number): void;

  /**
   * Configure font settings.
   * @param defaultFont  Path to default font file (in virtual FS), or null
   * @param defaultFamily  Default font family name
   */
  setFonts(defaultFont: string | null, defaultFamily: string): void;

  /**
   * Add a font file to the library.
   * @param name  Font filename (used for matching)
   * @param data  Font file data
   */
  addFont(name: string, data: Uint8Array): void;

  /**
   * Load a subtitle track from ASS/SSA content.
   * @param content  ASS file content as string or Uint8Array
   * @returns Track information
   */
  loadTrack(content: string | Uint8Array): TrackInfo;

  /**
   * Free the current subtitle track.
   */
  freeTrack(): void;

  /**
   * Render a subtitle frame at the given time.
   * @param timeMs  Time in milliseconds
   * @returns Render result with RGBA buffer
   */
  renderFrame(timeMs: number): RenderResult;

  /**
   * Set an image texture for a VSFilterMod channel.
   * The image will be tiled to fill the glyph area.
   *
   * @param channel  VSFModChannel (0-3)
   * @param imageData  PNG/JPEG/BMP image data, or null to clear
   */
  setChannelImage(channel: VSFModChannel, imageData: Uint8Array | null): void;

  /**
   * Write a file to the Emscripten virtual filesystem.
   * Used to provide images referenced by \Nimg tags.
   *
   * @param path  Virtual filesystem path
   * @param data  File data
   */
  writeFile(path: string, data: Uint8Array): void;

  /**
   * Read the current RGBA render buffer directly.
   * More efficient than renderFrame() if you just need the buffer pointer.
   */
  getRenderBuffer(): Uint8ClampedArray;
}

export default LibassRenderer;
