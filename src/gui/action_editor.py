"""Dialog for editing one action value, typed by its entry in action_spec.

Shared by Node and ActionPoint so the two context menus cannot drift apart.
"""

import logging

from PyQt6.QtWidgets import QInputDialog

from utilities import action_spec

logger = logging.getLogger(__name__)


def ask_action_value(view, screen_pos, name, current):
    """Show the editor for one action next to the item; None if cancelled."""
    spec = action_spec.spec_for(name)
    dialog = QInputDialog(view)
    dialog.setWindowTitle(f"Set {name}")
    dialog.setLabelText(spec.prompt or f"Enter {name} value:")
    dialog.move(screen_pos)

    if spec.kind == action_spec.CHOICE:
        labels = [label for label, _ in spec.choices]
        current_label = spec.label_for(current)
        if current_label not in labels:
            # An unrecognised value stays selectable rather than being silently
            # rounded to a code the firmware knows
            labels.append(current_label)
        dialog.setComboBoxItems(labels)
        dialog.setTextValue(current_label)
        if dialog.exec() != QInputDialog.DialogCode.Accepted:
            return None
        try:
            return spec.value_for(dialog.textValue())
        except KeyError:
            return current

    dialog.setInputMode(QInputDialog.InputMode.DoubleInput)
    dialog.setDoubleRange(spec.minimum, spec.maximum)
    dialog.setDoubleDecimals(spec.decimals)
    dialog.setDoubleValue(current)
    if dialog.exec() != QInputDialog.DialogCode.Accepted:
        return None
    return dialog.doubleValue()
