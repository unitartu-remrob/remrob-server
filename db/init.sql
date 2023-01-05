CREATE TABLE bookings (
    id SERIAL NOT NULL, 
    user_id VARCHAR(100), 
    simulation BOOLEAN, 
    project VARCHAR(100), 
    start_time VARCHAR(100), 
    end_time VARCHAR(100), 
    admin VARCHAR(255), 
    activated BOOLEAN, 
    PRIMARY KEY (id)
);

CREATE TABLE cameras (
    id SERIAL NOT NULL, 
    cell INTEGER, 
    camera_ip VARCHAR(100), 
    PRIMARY KEY (id)
);

CREATE TABLE inventory (
    id SERIAL NOT NULL, 
    robot_id INTEGER, 
    slug VARCHAR(100), 
    project VARCHAR(100), 
    cell INTEGER, 
    status BOOLEAN, 
    end_time VARCHAR(100), 
    "user" INTEGER, 
    vnc_uri VARCHAR(255), 
    issue BOOLEAN, 
    PRIMARY KEY (id)
);

CREATE TABLE simulation_containers (
    id SERIAL NOT NULL, 
    container_id INTEGER, 
    slug VARCHAR(100), 
    end_time VARCHAR(100), 
    "user" INTEGER, 
    vnc_uri VARCHAR(255), 
    PRIMARY KEY (id)
);
INSERT INTO simulation_containers (container_id, slug) VALUES (1, 'robosim-1');
INSERT INTO simulation_containers (container_id, slug) VALUES (2, 'robosim-2');
INSERT INTO simulation_containers (container_id, slug) VALUES (3, 'robosim-3');
INSERT INTO simulation_containers (container_id, slug) VALUES (4, 'robosim-4');
INSERT INTO simulation_containers (container_id, slug) VALUES (5, 'robosim-5');
INSERT INTO simulation_containers (container_id, slug) VALUES (6, 'robosim-6');
INSERT INTO simulation_containers (container_id, slug) VALUES (7, 'robosim-7');
INSERT INTO simulation_containers (container_id, slug) VALUES (8, 'robosim-8');
INSERT INTO simulation_containers (container_id, slug) VALUES (9, 'robosim-9');


CREATE TABLE token_blocklist (
    id SERIAL NOT NULL, 
    jti VARCHAR(36) NOT NULL, 
    created_at TIMESTAMP WITHOUT TIME ZONE NOT NULL, 
    PRIMARY KEY (id)
);

CREATE INDEX ix_token_blocklist_jti ON token_blocklist (jti);

CREATE TABLE "user" (
    id SERIAL NOT NULL, 
    first_name VARCHAR(255), 
    last_name VARCHAR(255), 
    email VARCHAR(100), 
    password VARCHAR(255), 
    active BOOLEAN, 
    role VARCHAR(100), 
    PRIMARY KEY (id), 
    UNIQUE (email)
);

INSERT INTO "user" (first_name, last_name, email, password, active, role)
VALUES ('Jerry', 'Platypus', 'admin', '$2b$12$sa40C6xxBzGkiGf8ZUHBPuzHA0yTSBsLA.FDCdldJdYonIr5PNVYC', TRUE, 'ROLE_ADMIN');