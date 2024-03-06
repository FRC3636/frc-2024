{ config, pkgs, lib, ... }:

with lib;
let
  cfg = config.services.photonvision;
in
{
  options = {
    services.photonvision = {
      enable = mkEnableOption (mdDoc "PhotonVision, an open source vision solution for the _FIRST_ Robotics Competition.");

      package = mkOption {
        type = types.package;
        default = pkgs.photonvision;
        description = "The PhotonVision package to use.";
      };
    };
  };

  config = mkIf cfg.enable {
    systemd.services.photonvision = {
      description = "PhotonVision, an open source vision solution for the FIRST Robotics Competition.";

      wantedBy = [ "multi-user.target" ];
      after = [ "network.target" ];

      serviceConfig = {
        ExecStart = "${cfg.package}/bin/photonvision";

        # ephemeral root directory
        RuntimeDirectory = "photonvision";
        RootDirectory = "/run/photonvision";

        # setup persistent state and logs directories
        StateDirectory = "photonvision";
        LogsDirectory = "photonvision";

        BindReadOnlyPaths = [
          # mount the nix store read-only
          "/nix/store"

          # the JRE reads the user.home property from /etc/passwd
          "/etc/passwd"
        ];
        BindPaths = [
          # mount the configuration and logs directories to the host
          "/var/lib/photonvision:/photonvision_config"
          "/var/log/photonvision:/photonvision_config/logs"
        ];

        # for PhotonVision's dynamic libraries, which it writes to /tmp
        PrivateTmp = true;
      };
    };

    networking.firewall = {
      allowedTCPPorts = [ 5800 ];
      allowedTCPPortRanges = [ { from = 1180; to = 1190; } ];
    };
  };
}
