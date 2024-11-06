{ config, pkgs, lib, ... }:

with lib;
let
  cfg = config.services.pplib-coprocessor;
in
{
  options = {
    services.pplib-coprocessor = {
      enable = mkEnableOption (mdDoc "Offload PathPlannerLib's pathfinding to a coprocessor");
      package = mkPackageOption pkgs "pplib-coprocessor" {};
      server = mkOption {
        type = types.str;
        default = "localhost";
        description = "The hostname or address of the PPLib server.";
      };
    };
  };

  config = mkIf cfg.enable {
    systemd.services.pplib-coprocessor = {
      description = "Offload PathPlannerLib's pathfinding to a coprocessor";

      wantedBy = [ "multi-user.target" ];
      after = [ "network.target" ];

      serviceConfig = {
        Type = "simple";
        ExecStart = "${cfg.package}/bin/pplib-coprocessor ${cfg.server}";

        Restart = "on-failure";
        RestartSec = "1";
      };
    };

    networking.firewall = {
      allowedTCPPorts = [ 5800 ];
      allowedTCPPortRanges = [ { from = 1180; to = 1190; } ];
    };
  };
}
