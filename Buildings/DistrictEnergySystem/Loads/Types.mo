within Buildings.DistrictEnergySystem.Loads;
package Types "Package with type definitions"
  extends Modelica.Icons.TypesPackage;

  type ModelType = enumeration(
      HeatPort
      "Thermal model with heat port",
      ODE
      "Temperature based on first order ODE",
      PrescribedT
      "Prescribed temperature")
    "Enumeration to define the type of load model"
  annotation(Documentation(info="<html>
    <p>
    Enumeration to define the type of load model.
    </p>
    </html>", revisions=
    "<html>
    <ul>
    <li>
    June 25, 2019, by Antoine Gautier:<br/>
    First implementation.
    </li>
    </ul>
    </html>"));
end Types;
