{
  description = "Coprocessor NixOS configurations for 3636's 2024 FRC robot.";

  inputs = {
    nixos-rk3588.url = "github:ryan4yin/nixos-rk3588";
  };

  outputs = { self, nixos-rk3588, }:
    let
      inherit (nixos-rk3588.inputs) nixpkgs;
      inherit (nixpkgs) lib;
    in
    {
      colmena = {
        meta = {
          nixpkgs = import nixpkgs {
            system = "x86_64-linux";
            overlays = [];
          };
          specialArgs = {
            rk3588 = nixos-rk3588.inputs;
          };
        };

        tangerine = {
          deployment = {
            targetHost = "10.36.36.10";
            targetUser = "root";
          };

          imports = [
            ./modules
            ./hosts/tangerine
          ];
        };

        boop = {
          deployment = {
            targetHost = "10.36.36.11";
            targetUser = "root";
          };

          nixpkgs.crossSystem.config = "aarch64-unknown-linux-gnu";

          imports = [
            nixos-rk3588.nixosModules.orangepi5

            ./modules
            ./hosts/boop
          ];
        };
      };

      devShell = lib.attrsets.genAttrs
        [
          "x86_64-linux"
          "aarch64-linux"
          "x86_64-darwin"
          "aarch64-darwin"
        ]
        (system:
          let
            pkgs = nixpkgs.legacyPackages.${system};
          in
          pkgs.mkShell {
            name = "3636-coprocessors";
            buildInputs = with pkgs; [
              colmena
            ];
          });
    };
}
