compose_dir := "dddmr_docker"

start-rviz:
    xhost +local:docker
    docker compose -f {{compose_dir}}/compose.laptop.yaml down
    docker compose -f {{compose_dir}}/compose.laptop.yaml up

start-dddmr:
    docker compose -f {{compose_dir}}/compose.robot.yaml down
    docker compose -f {{compose_dir}}/compose.robot.yaml up
