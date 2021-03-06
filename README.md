#  <span style="color:#F3B73B">SVG Interface to JSON IR Tool</span>

SVG interfaces (```.mi``` files) are a simple approach to specifying control
interfaces for Digitial Musical Instruments or DMIs. SVG Interfaces generate OSC 
control messages and as such as indepdendent of any particular audio engine and were
designed within the context of the [Muses](https://muses-dmi.github.io/) project.

The SVG Interface
[documentation](https://github.com/muses-dmi/svg-creator/blob/master/docs/interfaces.md)
provides details of the spefication and how to create them. The tool descibed
here converts a valid SVG interface to a particular backend's intermediate
representation. (The backend in question is explictly specified in the input SVG
interface.)

As noted this tool is part of the larger [Muses](https://muses-dmi.github.io/)
project and is a component within a larger set of tools for working with SVG
interfaces.

#  <span style="color:#F3B73B">Dependencies</span> 

The application is written in [Rust](https://www.rust-lang.org/) and tested with
1.34.1 (and nightly). To install Rust go you need simply to install
[Rustup](https://rustup.rs/) and if you already have Rust installed, then you can update
with the command ```rustup update```.

#  <span style="color:#F3B73B">Building</span>

To build run the command:

```
cargo build --release
```

#  <span style="color:#F3B73B">Using it</span>

To run the application and see its help screen simply run the command

```
cargo run --release
```

```
Usage: target/release/interfaces FILE [options]

Options:
        --png NAME      generate PNG output
        --json NAME     JSON file output
        --illustrator   SVG used Adobe Illustrator attribute encoding
    -h, --help          print this help menu
```

To test let's compile ```examples/lightpad.svg``` using the command:

```
cargo run --release -- examples/lightpad.svg
```

which outputs the following JSON:

```json
{"buffer":[[1,1,1,0,5,5,5,5,5,5,5,5,5,5,5],[1,1,1,0,5,5,5,5,5,5,5,5,5,5,5],[1,1,1,0,5,5,5,5,5,5,5,5,5,5,5],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[2,2,2,0,6,6,6,6,6,6,6,6,6,6,6],[2,2,2,0,6,6,6,6,6,6,6,6,6,6,6],[2,2,2,0,6,6,6,6,6,6,6,6,6,6,6],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[3,3,3,0,7,7,7,7,7,7,7,7,7,7,7],[3,3,3,0,7,7,7,7,7,7,7,7,7,7,7],[3,3,3,0,7,7,7,7,7,7,7,7,7,7,7],[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],[4,4,4,0,8,8,8,8,8,8,8,8,8,8,8],[4,4,4,0,8,8,8,8,8,8,8,8,8,8,8],[4,4,4,0,8,8,8,8,8,8,8,8,8,8,8]],"controllers":[{"address":"/midicc","args":[100],"id":1,"rgb":"rgb(217,137,188)","type_id":"pad"},{"address":"/midicc","args":[101],"id":2,"rgb":"rgb(217,137,188)","type_id":"pad"},{"address":"/midicc","args":[102],"id":3,"rgb":"rgb(217,137,188)","type_id":"pad"},{"address":"/midicc","args":[103],"id":4,"rgb":"rgb(217,137,188)","type_id":"pad"},{"address":"/midicc","args":[104],"id":5,"max":127,"min":0,"rgb":"rgb(96,95,164)","type_id":"vert_slider"},{"address":"/midicc","args":[105],"id":6,"max":127,"min":0,"rgb":"rgb(96,95,164)","type_id":"vert_slider"},{"address":"/midicc","args":[106],"id":7,"max":127,"min":0,"rgb":"rgb(96,95,164)","type_id":"vert_slider"},{"address":"/midicc","args":[107],"id":8,"max":127,"min":0,"rgb":"rgb(96,95,164)","type_id":"vert_slider"}],"interface":"lightpad"}
```

The resulting JSON can be pretty printed with a command such as [jsonpp](https://github.com/jmhodges/jsonpp), but in general it would be generated direclty into a file, the comamnd line option ```--json <filename>``` or redirecting ```stdout```.

#  <span style="color:#F3B73B">More Information</span>

Parent project

   - [Muses](https://muses-dmi.github.io/).

Tool and documentation for specification of interfaces as SVGs:

   - [SVG Creator tool](https://github.com/muses-dmi/svg-creator).
   - [SVG Interface Documentation](https://github.com/muses-dmi/svg-creator/blob/master/docs/interfaces.md).

Tools for translating SVG Interfaces to the JSON intermidiate representation and different backends:

   - [SVG Interface to IR tool](https://github.com/muses-dmi/svg_interface). (This repo.)
   - [Interface IR to Littlefoot tool](https://github.com/muses-dmi/svg-littlefoot).
   - [SVG Sensel Driver](https://github.com/muses-dmi/sensel_osc).


#  <span style="color:#F3B73B">License</span>

The source in this repo is licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)
 * [Mozilla Public License 2.0](https://www.mozilla.org/en-US/MPL/2.0/)

at your option.

Dual MIT/Apache2 is strictly more permissive