document.addEventListener('DOMContentLoaded', () => {
    // Elements
    const loginSection = document.getElementById('login_section');
    const dashboardSection = document.getElementById('dashboard_section');
    const vendorDashboard = document.getElementById('vendor_dashboard');
    const customerDashboard = document.getElementById('customer_dashboard');

    const loginButton = document.getElementById('login_button'); // Single login button
    const logoutButton = document.getElementById('logout_button'); // Single logout button

    const tableSelectVendor = document.getElementById('table_select_vendor');
    const tableSelectCustomer = document.getElementById('table_select_customer');

    let currentUserType = ''; // Tracks if the user is vendor or customer
    const vendorCredentials = { username: "vendor", password: "vendor123" };
    const customerCredentials = { username: "customer", password: "customer123" };

    let totalTables = 4; // Initial table count
    let tableConfirmations = {}; // Confirmation data for tables
    let kitchenConfirmations = {}; // Kitchen readiness for each table
    let taskQueue = []; // Task queue for orders

    // Function to dynamically generate table options for dropdowns
    function generateTableOptions() {
        const vendorSelect = document.getElementById('table_select_vendor');
        const customerSelect = document.getElementById('table_select_customer');

        // Clear existing options
        vendorSelect.innerHTML = '';
        customerSelect.innerHTML = '';

        for (let i = 1; i <= totalTables; i++) {
            const vendorOption = document.createElement('option');
            vendorOption.value = i;
            vendorOption.textContent = `Table ${i}`; // No space between "Table" and number
            vendorSelect.appendChild(vendorOption);

            const customerOption = document.createElement('option');
            customerOption.value = i;
            customerOption.textContent = `Table ${i}`; // No space between "Table" and number
            customerSelect.appendChild(customerOption);
        }
    }

    // Login functionality (single button for both vendor and customer)
    loginButton.addEventListener('click', () => {
        const username = document.getElementById('username').value;
        const password = document.getElementById('password').value;

        if (username === vendorCredentials.username && password === vendorCredentials.password) {
            currentUserType = 'vendor';
            loginSection.style.display = 'none';
            dashboardSection.style.display = 'block';
            vendorDashboard.style.display = 'block';
            generateTableOptions();
        } else if (username === customerCredentials.username && password === customerCredentials.password) {
            currentUserType = 'customer';
            loginSection.style.display = 'none';
            dashboardSection.style.display = 'block';
            customerDashboard.style.display = 'block';
            generateTableOptions();
        } else {
            alert("Invalid Username or Password.");
        }
    });

    // Logout functionality (single button for both vendor and customer)
    logoutButton.addEventListener('click', () => {
        loginSection.style.display = 'block';
        dashboardSection.style.display = 'none';
        vendorDashboard.style.display = 'none';
        customerDashboard.style.display = 'none';
        document.getElementById('username').value = '';
        document.getElementById('password').value = '';
        currentUserType = '';

        // Clear table confirmations, task queue, and kitchen readiness on logout
        tableConfirmations = {};
        kitchenConfirmations = {};
        taskQueue = [];
        updateTableConfirmationDisplay(); // Reset confirmation UI
        updateTaskQueue(); // Reset task queue UI
        updateKitchenConfirmationDisplay(); // Reset kitchen readiness UI
    });

    // Vendor: Place order
    document.getElementById('place_order_button').addEventListener('click', () => {
        const tableNumber = tableSelectVendor.value;
        if (tableNumber) {
            const order = `Table${tableNumber}`; // Consistent format
            taskQueue.push(order);
            updateTaskQueue();

            // Send order to backend
            fetch('http://localhost:5000/order', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ order })
            })
            .then(response => response.json())
            .then(data => console.log(data))
            .catch(error => console.error('Error:', error));

            alert(`Order placed for Table${tableNumber}.`);
        } else {
            alert("Please select a table to place an order.");
        }
    });

    // Vendor: Cancel order
    document.getElementById('cancel_order_button').addEventListener('click', () => {
        const tableNumber = tableSelectVendor.value;
        if (tableNumber) {
            const orderToCancel = `Table${tableNumber}`; // Consistent format

            const index = taskQueue.indexOf(orderToCancel);
            if (index !== -1) {
                taskQueue.splice(index, 1);
                updateTaskQueue();

                // Send cancel request to backend
                fetch('http://localhost:5000/cancel', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ task: orderToCancel })
                })
                .then(response => response.json())
                .then(data => console.log(data))
                .catch(error => console.error('Error:', error));

                alert(`Order cancelled for Table${tableNumber}.`);
            } else {
                alert("No order found for the selected table.");
            }
        } else {
            alert("Please select a table to cancel the order.");
        }
    });

    // Vendor: Confirm table readiness (Push button configuration)
    document.getElementById('confirm_table_vendor_button').addEventListener('click', () => {
        const tableNumber = tableSelectVendor.value;
        if (tableNumber) {
            tableConfirmations[tableNumber] = true; // Set confirmation to true
            alert(`Table ${tableNumber} confirmed by vendor.`);

            // Remove the table from the task queue if it exists
            const orderToRemove = `Table${tableNumber}`;
            const index = taskQueue.indexOf(orderToRemove);
            if (index !== -1) {
                taskQueue.splice(index, 1); // Remove the confirmed table from the queue
                updateTaskQueue(); // Update the displayed task queue
            }

            // Send confirmation to backend
            fetch('http://localhost:5000/table_confirm', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(true),
            })
            .then(response => response.json())
            .then(data => console.log(data))
            .catch(error => console.error('Error:', error));

            // Reset confirmation to false after 2 seconds
            setTimeout(() => {
                tableConfirmations[tableNumber] = false;
                kitchenConfirmations[tableNumber] = false; // Reset kitchen readiness as well
                updateTableConfirmationDisplay(); // Update UI after resetting the confirmation
            }, 2000);

            updateTableConfirmationDisplay();
        } else {
            alert("Please select a table to confirm.");
        }
    });

    // Vendor: Confirm kitchen readiness for specific table (Push button configuration)
    document.getElementById('confirm_kitchen_button').addEventListener('click', () => {
        const tableNumber = tableSelectVendor.value;
        if (tableNumber) {
            kitchenConfirmations[tableNumber] = true; // Set kitchen readiness for the table
            alert(`Kitchen readiness confirmed for Table ${tableNumber}.`);

            // Send kitchen confirmation to backend
            fetch('http://localhost:5000/kitchen_confirm', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ table: tableNumber, ready: true }),
            })
            .then(response => response.json())
            .then(data => console.log(data))
            .catch(error => console.error('Error:', error));

            // Reset kitchen readiness after 2 seconds
            setTimeout(() => {
                kitchenConfirmations[tableNumber] = false;
                updateKitchenConfirmationDisplay(); // Update UI after resetting kitchen readiness
            }, 2000);

            updateKitchenConfirmationDisplay();
        } else {
            alert("Please select a table to confirm kitchen readiness.");
        }
    });

    // Vendor: Add new table
    document.getElementById('add_table_button').addEventListener('click', () => {
        totalTables += 1;
        generateTableOptions();
        alert(`Table ${totalTables} added.`);
    });

    // Vendor: Remove a table
    document.getElementById('remove_table_button').addEventListener('click', () => {
        const tableNumber = tableSelectVendor.value;
        if (tableNumber && totalTables > 1) {
            totalTables -= 1;
            generateTableOptions();
            alert(`Table ${tableNumber} removed.`);
        } else {
            alert("Cannot remove the last table or no table selected.");
        }
    });

    // Helper function to update the task queue display
    function updateTaskQueue() {
        const taskQueueDiv = document.getElementById('task_queue');
        taskQueueDiv.innerHTML = ''; // Clear existing tasks

        if (taskQueue.length === 0) {
            taskQueueDiv.innerHTML = "No tasks in the queue."; // Message when queue is empty
        }

        taskQueue.forEach(task => {
            const taskItem = document.createElement('div');
            taskItem.textContent = task;
            taskItem.classList.add('task-item');
            taskQueueDiv.appendChild(taskItem);
        });
    }

// Helper function to update the table confirmation display (vendor and customer)
function updateTableConfirmationDisplay() {
    const vendorConfirmationDiv = document.getElementById('vendor_table_confirmations');
    const customerConfirmationDiv = document.getElementById('table_confirmation_customer');

    // Clear the current confirmations
    vendorConfirmationDiv.innerHTML = '';
    customerConfirmationDiv.innerHTML = '';

    // Update vendor table confirmations
    for (let table in tableConfirmations) {
        const confirmationItem = document.createElement('div');
        confirmationItem.textContent = `Table ${table}: ${tableConfirmations[table] ? 'Confirmed' : 'Not Confirmed'}`;
        confirmationItem.style.color = tableConfirmations[table] ? 'green' : 'red'; // Green for confirmed, red for not confirmed
        vendorConfirmationDiv.appendChild(confirmationItem);
    }

    // Update customer table confirmations
    for (let table in tableConfirmations) {
        const confirmationItem = document.createElement('div');
        confirmationItem.textContent = `Table ${table}: ${tableConfirmations[table] ? 'Confirmed' : 'Not Confirmed'}`;
        confirmationItem.style.color = tableConfirmations[table] ? 'green' : 'red'; // Green for confirmed, red for not confirmed
        customerConfirmationDiv.appendChild(confirmationItem);
    }
}

    // Helper function to update the kitchen readiness display
    function updateKitchenConfirmationDisplay() {
        const kitchenConfirmationDiv = document.getElementById('kitchen_confirmation');
        kitchenConfirmationDiv.innerHTML = '';
        for (let table in kitchenConfirmations) {
            const status = kitchenConfirmations[table] ? 'Kitchen Ready' : 'Kitchen Not Ready';
            const color = kitchenConfirmations[table] ? 'green' : 'red';

            const kitchenConfirmationItem = document.createElement('div');
            kitchenConfirmationItem.textContent = `Table ${table}: ${status}`;
            kitchenConfirmationItem.style.color = color;
            kitchenConfirmationDiv.appendChild(kitchenConfirmationItem);
        }
    }

});
