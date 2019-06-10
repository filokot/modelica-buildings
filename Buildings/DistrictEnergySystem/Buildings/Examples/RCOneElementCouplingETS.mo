within Buildings.DistrictEnergySystem.Buildings.Examples;
model RCOneElementCouplingETS
  "Example illustrating the coupling of a RC building model and a ETS model"
  import Buildings;
  extends Modelica.Icons.Example;
  package Medium = Buildings.Media.Water "Fluid in the pipes";
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end RCOneElementCouplingETS;
