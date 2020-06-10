## Clang tooling libraries
### Tooling is currently being developed under a docker container:
- Build:
	```sh
	sudo docker build -t clang .
	```
- Run:
	```sh
	sudo docker run -it -v $PWD:/home clang
	```
	This sets the PWD as the home and launches the container
	(To exit and return to your shell: `[Ctrl+P Ctrl+Q]`)
- Docker image is available at: https://hub.docker.com/repository/docker/divmadan/tooling

### Makefile:
- Used for cpp based tooling, to compile `parse.cpp`
- Currently not being used, as Python bindings for clangtooling is being explored.

### libclang.py:
- Using python libclang to parse cpp code, for generation of metadata by static analysis.
- Run:
	```py
	python3 libclang.py <options>
	```