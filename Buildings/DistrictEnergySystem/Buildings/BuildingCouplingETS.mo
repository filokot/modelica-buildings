within Buildings.DistrictEnergySystem.Buildings;
model BuildingCouplingETS
  "Model for coupling a building model and an substation model"

  DistrictEnergySystem.Buildings.BaseClasses.HeatingOrCooling heaOrCoo
    annotation (Placement(transformation(extent={{-10,28},{10,48}})));
    annotation (defaultComponentName="buiCou",
              Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end BuildingCouplingETS;
