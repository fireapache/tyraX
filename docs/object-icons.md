# Object icons (editor badges)

An object icon is a small round badge the editor draws above an object in the
viewport, so a marker says what it is from across the map: a flag over a
capture area, a tank over the spot a tank spawns on, a person over a spawn
point. It lives in the editor and nowhere else - nothing about an icon is
generated, baked, compiled or shipped.

Select an object and choose **Properties > Editor icon**. `<none>` removes it.

## What you see

- **A badge in the object's own colour.** Give each team's markers their team
  colour and the map reads at a glance: who, by the colour; what, by the glyph.
  The glyph is drawn dark on a light colour and light on a dark one.
- **Always the same size.** The badge is screen-space, like a
  [comment](comments.md)'s icon: a marker ten metres away and one across the
  map read identically, and neither grows into the scene when you zoom in.
- **On a stem above the object.** The badge floats over the object's point with
  a thin stem down to it, so it never hides the thing it labels - and a marker
  standing inside a model (a spawn spot under the vehicle parked on it) is
  still visible and clickable.
- **The name on hover and while selected**, beside the badge.
- **Click it to select the object.** A badge is hit in screen space and wins
  over whatever is under it, exactly like a comment's icon; the right-click
  stack lists it first.

An object on a hidden layer, or hidden with its own eye in the Outliner, shows
no badge.

## The icons

| Name | Reads as |
|---|---|
| `flag` | an objective, a capture area |
| `person` | a spawn point for people |
| `tank` | a tank, or where one stands |
| `helicopter` | a helicopter, or where one stands |
| `car` | a car, or where one stands |
| `house` | a base, a headquarters |
| `crate` | a supply point, a pickup |
| `target` | a point of interest |
| `dot` | anything else |

The glyphs are drawn by the editor (it ships no icon assets), so they follow
the UI scale. A comment and a particle emitter already are icons and take none.

## Format

`"editorIcon": "<name>"` in the object's `objects/<id>.json`, written only when
set (format 123). It takes part in object equality, undo and collaboration like
every other object field, and has no PS2 runtime cost: no generated file reads
it. A name this editor does not know draws the `dot`.
