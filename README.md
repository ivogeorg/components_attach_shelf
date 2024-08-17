### `components_attach_shelf`

This was supposed to be a branch of `attach_shelf` under a `checkpoint9`->[multiple packages] structure, which doesn't make sense.

Anyway, this is a rework of [attach_shelf](https://github.com/ivogeorg/attach_shelf.git) into ROS2 components. The requirements are in [checkpoint-10](assets/checkpoint-10.pdf).

#### Implementation notes

1. The original multi-callback [`approach_service_server.cpp`](https://github.com/ivogeorg/attach_shelf/blob/main/src/approach_service_server.cpp) does not work as a monolithic component. Have to understand how multi-callback nodes work as components. _It's not impossible that the monolithic node has to be broken down into other nodes, though I can't see how - the server node still has to subscribe to all the topics it needs and listen to the tranforms it needs._