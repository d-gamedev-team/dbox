# dbox

![dbox](https://raw.github.com/d-gamedev-team/dbox/master/screenshot/work_in_progress.png)

![dbox](https://raw.github.com/d-gamedev-team/dbox/master/screenshot/dbox.png)

**This is a work in progress, don't use yet until all runtime issues are resolved.**

This is a D port of the [Box2D] game physics library.

Currently dbox targets Box2D version **2.3.1**.

[Box2D] was created by [Erin Catto].

Homepage: https://github.com/d-gamedev-team/dbox

## Examples

Use [dub] to build and run the examples:

```
$ dub run dbox:hello_world

$ dub run dbox:demo
```

Note: You will need to install the [glfw] shared library in order to run the example.

## Documentation

Documentation will be generated soon.

## Building dbox as a static library

Run [dub] alone in the root project directory to build **dbox** as a static library:

```
$ dub
```

## Links

- The [Box2D homepage][Box2D].
- The [Box2D repository][Box2D_Repo].

## License

Distributed under the [zlib] license.

See the accompanying file [license.txt][zlib].

[Erin Catto]: http://www.gphysics.com
[dub]: http://code.dlang.org/
[Box2D]: http://box2d.org/
[Box2D_Repo]: http://code.google.com/p/box2d/
[zlib]: https://raw.github.com/d-gamedev-team/dbox/master/license.txt
[glfw]: http://www.glfw.org/
