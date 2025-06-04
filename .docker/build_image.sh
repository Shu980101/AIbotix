
echo -e "Building image moveit2_introduction:latest"

DOCKER_BUILDKIT=1 \
docker build --pull --rm -f ./.docker/Dockerfile \
--build-arg BUILDKIT_INLINE_CACHE=1 \
--target base \
--tag moveit2_introduction:latest .