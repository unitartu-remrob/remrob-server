module.exports = {
  apps : [{
    name   : "remrob",
    script : "./bin/www",
    instances: "4",
    exec_mode: "cluster",
    env_production: {
      NODE_ENV: "production"
    },
    env_development: {
      NODE_ENV: "development"
    }
  }]
}
