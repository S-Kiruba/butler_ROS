class ButlerTester:
    def __init__(self):
        rospy.init_node("butler_tester", anonymous=True)

        # Publishers
        self.order_pub = rospy.Publisher("/new_order", String, queue_size=10)
        self.cancel_pub = rospy.Publisher("/cancel_task", String, queue_size=10)
        self.kitchen_confirm_pub = rospy.Publisher("/kitchen_confirm", Bool, queue_size=10)
        self.table_confirm_pub = rospy.Publisher("/table_confirm", Bool, queue_size=10)

        # Initialize variables
        self.running = True

    def handle_input(self):
        """Handle keyboard input."""
        while self.running:
            key = input("\nEnter commands: ")

            if key == "q":
                self.running = False
                print("\nExiting...")
                break

            elif key.isdigit():
                # Handle multiple orders (e.g., "123" for table1, table2, table3)
                for table in key:
                    table_num = f"table{table}"
                    self.order_pub.publish(table_num)
                    print(f"\nSent new order for {table_num}")

            elif key == "k":
                self.kitchen_confirm_pub.publish(True)
                print("\nSent kitchen confirmation (True)")

            elif key == "t":
                self.table_confirm_pub.publish(True)
                print("\nSent table confirmation (True)")

            elif key == "h":
                self.order_pub.publish("home")
                print("\nSent robot to home")

            elif key == "m":
                self.print_menu()

            elif key.startswith("c") and key[1:].isdigit():
                # Handle multiple cancel commands (e.g., "c123" to cancel table1, table2, table3)
                for table in key[1:]:
                    table_num = f"table{table}"
                    self.cancel_pub.publish(table_num)
                    print(f"\nCancelled order for {table_num}")

            else:
                print("\nInvalid input. Press 'm' to show the menu.")

    def print_menu(self):
        """Print the menu of available commands."""
        print("\n" + "=" * 40)
        print("          BUTLER ROBOT TEST INTERFACE")
        print("=" * 40)
        print("COMMANDS:")
        print("  Orders:")
        print("    1-5       : Send order to Table 1-5")
        print("                (e.g., '123' for Table 1, Table 2, and Table 3)")
        print("")
        print("  Cancellations:")
        print("    c1-c5     : Cancel order for Table 1-5")
        print("                (e.g., 'c123' for Table 1, Table 2, and Table 3)")
        print("")
        print("  Confirmations:")
        print("    k         : Send kitchen confirmation (True)")
        print("    t         : Send table confirmation (True)")
        print("")
        print("  Navigation:")
        print("    h         : Return robot to home")
        print("")
        print("  Other:")
        print("    m         : Show this menu")
        print("    q         : Quit the program")
        print("=" * 40)

    def run(self):
        """Main run function."""
        self.print_menu()
        self.handle_input()
        print("Test interface shutdown complete")


def main():
    try:
        tester = ButlerTester()
        tester.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
