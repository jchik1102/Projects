# Connect Ignition to OpenPLC

This is the communication setup only. HMI screen construction remains the next Ignition work package.

## Device connection

1. Start the OpenPLC Runtime and PLC program.
2. In the Ignition Gateway, create a **Modbus TCP** device connection.
3. Use:
   - Name: `OpenPLC`
   - Hostname: `127.0.0.1`
   - Port: `5020`
   - Unit/Server ID: `1`
4. Confirm the device reports Connected.

Modbus devices do not expose browseable symbolic tags, so create OPC tags using the addresses in `OpenPLC_Register_Map.csv`.

## Example OPC item paths

| Ignition tag | OPC item path | Access |
|---|---|---|
| Raw tank level raw | `[OpenPLC]HR1` | Read/write register, used read-only by HMI |
| Pressure PV raw | `[OpenPLC]HR8` | Read-only by HMI |
| Pressure setpoint raw | `[OpenPLC]HR201` | Read/write |
| Batch state | `[OpenPLC]HR301` | Read-only |
| Auto mode request | `[OpenPLC]C104` | Momentary write |
| Start request | `[OpenPLC]C101` | Momentary write |
| Stop request | `[OpenPLC]C102` | Momentary write |
| Communication healthy | `[OpenPLC]C151` | Read-only |
| P-301A failed start | `[OpenPLC]C172` | Read-only |

## Scaling expressions

Use derived/expression tags or transform bindings:

- Level `%` = raw register / 10
- Flow `L/s` = raw register / 10
- Pressure `kPa` = raw register / 10
- Concentration `mg/L` = raw register / 100
- Speed/output `%` = raw register / 10

## Safe command pattern

Buttons should pulse request coils for approximately 250-500 ms:

- C101 Start
- C102 Stop
- C103 Reset
- C104 Auto mode
- C105 Manual mode
- C106 Maintenance mode

Setpoints:

- HR201 Pressure SP in kPa x10
- HR202 Concentration SP in mg/L x100
- HR203 Batch fill SP in % x10
- HR204 Mix time in seconds

Do not write PLC-owned commands C51-C57 or HR101-HR106 from Ignition.
