{
  description = "Coprocessor NixOS configurations for 3636's 2024 FRC robot.";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-23.11";
    nixos-rk3588 = {
      url = "github:ryan4yin/nixos-rk3588";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    rgb-2024 = {
      url = "github:FRC3636/rgb-2024";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    nixos-hardware.url = "github:NixOS/nixos-hardware/master";
  };

  outputs = { self, nixpkgs, nixos-rk3588, nixos-hardware, rgb-2024, }:
    let inherit (nixpkgs) lib;
    in rec {
      colmena = {
        meta = {
          nixpkgs = import nixpkgs {
            system = "x86_64-linux";
            overlays = [ overlays.default ];
          };
          specialArgs = { rk3588 = nixos-rk3588.inputs; };
        };

        # RGB LED strips Raspberry Pi
        copepod = {
          deployment = {
            targetHost = "10.36.36.20";
            targetUser = "root";
          };

          imports = [
            nixos-hardware.nixosModules.raspberry-pi-4

            ./modules
            ./hosts/copepod
          ];
        };

        # Mini PC
        tangerine = {
          deployment = {
            targetHost = "10.36.36.10";
            targetUser = "root";
          };

          imports = [ ./modules ./hosts/tangerine ];
        };

        # Photon vision Orange Pi
        boop = {
          deployment = {
            targetHost = "10.36.36.11";
            targetUser = "root";
          };

          nixpkgs = { system = "aarch64-linux"; };

          imports = [
            nixos-rk3588.nixosModules.orangepi5

            ./modules
            ./hosts/boop
          ];
        };
      };

      overlays.default = self: super: {
        photonvision = self.callPackage ./pkgs/photonvision.nix { };
        rgb-2024 = rgb-2024.packages.${self.system}.default;
      };

      devShell = lib.attrsets.genAttrs [
        "x86_64-linux"
        "aarch64-linux"
        "x86_64-darwin"
        "aarch64-darwin"
      ] (system:
        let pkgs = nixpkgs.legacyPackages.${system};
        in pkgs.mkShell {
          name = "3636-coprocessors";
          buildInputs = [ pkgs.colmena ];
        });
    };
}
