compose_dir := "dddmr_docker"

start-rviz:
    xhost +local:docker
    docker compose -f {{compose_dir}}/compose.laptop.yaml down
    docker compose -f {{compose_dir}}/compose.laptop.yaml up

start-dddmr:
    docker compose -f {{compose_dir}}/compose.robot.yaml down
    docker compose -f {{compose_dir}}/compose.robot.yaml up

start-mapping:
    docker compose -f {{compose_dir}}/compose.robot.mapping.yaml down
    docker compose -f {{compose_dir}}/compose.robot.mapping.yaml up

start-localization:
    docker compose -f {{compose_dir}}/compose.robot.localization.yaml down
    docker compose -f {{compose_dir}}/compose.robot.localization.yaml up

start-rviz-mapping:
    xhost +local:docker
    docker compose -f {{compose_dir}}/compose.laptop.mapping.yaml down
    docker compose -f {{compose_dir}}/compose.laptop.mapping.yaml up

start-rviz-localization:
    xhost +local:docker
    docker compose -f {{compose_dir}}/compose.laptop.localization.yaml down
    docker compose -f {{compose_dir}}/compose.laptop.localization.yaml up
