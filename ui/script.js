document.addEventListener('DOMContentLoaded', () => {
    // Elements
    const loginSection = document.getElementById('login_section');
    const dashboardSection = document.getElementById('dashboard_section');
    const vendorDashboard = document.getElementById('vendor_dashboard');
    const customerDashboard = document.getElementById('customer_dashboard');
    const loginButton = document.getElementById('login_button');
    const logoutButton = document.getElementById('logout_button');
    const tableSelectVendor = document.getElementById('table_select_vendor');
    const tableSelectCustomer = document.getElementById('table_select_customer');

    let currentUserType = ''; // Tracks user type (vendor or customer)
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

        // Generate table options for both vendor and customer
        for (let i = 1; i <= totalTables; i++) {
            const vendorOption = document.createElement('option');
            vendorOption.value = i;
            vendorOption.textContent = `Table${i}`;
            vendorSelect.appendChild(vendorOption);

            const customerOption = document.createElement('option');
            customerOption.value = i;
            customerOption.textContent = `Table${i}`;
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
            customerDashboard.style.display = 'none';
            generateTableOptions();
        } else if (username === customerCredentials.username && password === customerCredentials.password) {
            currentUserType = 'customer';
            loginSection.style.display = 'none';
            dashboardSection.style.display = 'block';
            vendorDashboard.style.display = 'none';
            customerDashboard.style.display = 'block';
            generateTableOptions();
        } else {
            alert("Invalid Username or Password.");
        }
    });

    // Logout functionality
    logoutButton.addEventListener('click', () => {
        loginSection.style.display = 'block';
        dashboardSection.style.display = 'none';
        vendorDashboard.style.display = 'none';
        customerDashboard.style.display = 'none';
        document.getElementById('username').value = '';
        document.getElementById('password').value = '';
        currentUserType = '';

        // Clear confirmations, task queue, and kitchen readiness on logout
        tableConfirmations = {};
        kitchenConfirmations = {};
        taskQueue = [];
        updateTableConfirmationDisplay();
        updateTaskQueue();
        updateKitchenConfirmationDisplay();
    });

    // Add task to queue
    function addTaskToQueue(tableNumber, actionBy) {
        if (tableNumber) {
            const task = { table: `Table${tableNumber}`, actionBy };
            taskQueue.push(task);
            updateTaskQueue();

            // Simulate backend request
            fetch('http://localhost:5000/order', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(task)
            })
                .then(response => response.json())
                .then(data => console.log(data))
                .catch(error => console.error('Error:', error));

            alert(`${actionBy}: Order placed for Table ${tableNumber}.`);
        } else {
            alert("Please select a table to place an order.");
        }
    }

    // Remove task from queue
    function removeTaskFromQueue(tableNumber, actionBy) {
        if (tableNumber) {
            const taskIndex = taskQueue.findIndex(
                (task) => task.table === `Table${tableNumber}`
            );

            if (taskIndex !== -1) {
                const removedTask = taskQueue.splice(taskIndex, 1);
                updateTaskQueue();

                // Simulate backend request
                fetch('http://localhost:5000/cancel', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ task: removedTask })
                })
                    .then(response => response.json())
                    .then(data => console.log(data))
                    .catch(error => console.error('Error:', error));

                alert(`${actionBy}: Order cancelled for Table ${tableNumber}.`);
            } else {
                alert("No order found for the selected table.");
            }
        } else {
            alert("Please select a table to cancel the order.");
        }
    }

    // Update task queue display
    function updateTaskQueue() {
        const taskQueueDiv = document.getElementById('task_queue');
        taskQueueDiv.innerHTML = '';

        if (taskQueue.length === 0) {
            taskQueueDiv.innerHTML = "No tasks in the queue.";
        }

        taskQueue.forEach((task) => {
            const taskItem = document.createElement('div');
            taskItem.textContent = `${task.table}`;
            taskQueueDiv.appendChild(taskItem);
        });
    }

// Assuming you have a button with id 'place_order_button' and a dropdown/select for table selection (id 'tableSelectVendor')
// Also assuming similar buttons for canceling, confirming table readiness, and kitchen readiness


// Place order functionality for Vendor
document.getElementById('place_order_button_vendor').addEventListener('click', () => {
    const tableNumber = tableSelectVendor.value;
    if (tableNumber) {
        const task = { table: `Table${tableNumber}` };  // Remove actionBy
        taskQueue.push(task);
        updateTaskQueue();

        // Send the order to the backend as an object
        fetch('http://localhost:5000/order', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ order: `table${tableNumber}` })
        })
        .then(response => response.json())
        .then(data => console.log(data))
        .catch(error => console.error('Error:', error));

        alert(`Order placed for Table ${tableNumber} by Vendor.`);
    } else {
        alert("Please select a table to place an order.");
    }
});

// Place order for Customer
document.getElementById('place_order_button_customer').addEventListener('click', () => {
    const tableNumber = tableSelectCustomer.value;
    if (tableNumber) {
        const task = { table: `Table${tableNumber}` };  // Remove actionBy
        taskQueue.push(task);
        updateTaskQueue();

        // Send the order to the backend as an object
        fetch('http://localhost:5000/order', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ order: `table${tableNumber}` })
        })
        .then(response => response.json())
        .then(data => console.log(data))
        .catch(error => console.error('Error:', error));

        alert(`Order placed for Table ${tableNumber} by Customer.`);
    } else {
        alert("Please select a table to place an order.");
    }
});



// Cancel order functionality for Vendor
document.getElementById('cancel_order_button_vendor').addEventListener('click', () => {
    const tableNumber = tableSelectVendor.value;
    if (tableNumber) {
        // Find the task in the queue
        const taskIndex = taskQueue.findIndex(task => task.table === `Table${tableNumber}`);
        if (taskIndex !== -1) {
            const removedTask = taskQueue.splice(taskIndex, 1)[0]; // Remove task from queue
            updateTaskQueue();

            // Send cancel request to backend
            fetch('http://localhost:5000/cancel', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ order: removedTask.table.toLowerCase() })  // Send the removed task object
            })
            .then(response => response.json())
            .then(data => console.log(data))
            .catch(error => console.error('Error:', error));

            alert(`Order cancelled for ${removedTask.table} by Vendor.`);
        } else {
            alert("No order found for the selected table.");
        }
    } else {
        alert("Please select a table to cancel the order.");
    }
});


// Cancel order for Customer
document.getElementById('cancel_order_button_customer').addEventListener('click', () => {
    const tableNumber = tableSelectCustomer.value;
    if (tableNumber) {
        // Find the task in the queue
        const taskIndex = taskQueue.findIndex(task => task.table === `Table${tableNumber}`);
        if (taskIndex !== -1) {
            const removedTask = taskQueue.splice(taskIndex, 1)[0]; // Remove task from queue and get the object
            updateTaskQueue();

            // Convert only the JSON payload to lowercase
            fetch('http://localhost:5000/cancel', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ order: removedTask.table.toLowerCase() }) // Lowercase only for JSON
            })
            .then(response => response.json())
            .then(data => console.log(data))
            .catch(error => console.error('Error:', error));

            // Alert with original formatting
            alert(`Order cancelled for ${removedTask.table} by Customer.`);
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
            const tableConfirmation = true; // Send "true" as a string for confirmation

            // Send confirmation to backend as "true"
            fetch('http://localhost:5000/table_confirm', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ confirmed: tableConfirmation }) // Send as "true"
            })
            .then(response => response.json())
            .then(data => console.log(data))
            .catch(error => console.error('Error:', error));

            alert(`Table ${tableNumber} confirmed by vendor.`);
        } else {
            alert("Please select a table to confirm.");
        }
    });

    // Vendor: Confirm kitchen readiness for specific table
    document.getElementById('confirm_kitchen_button').addEventListener('click', () => {
        const tableNumber = tableSelectVendor.value;
        if (tableNumber) {
            const kitchenConfirmation = true; // Send "true" as a string for kitchen confirmation

            // Send kitchen confirmation to backend as "true"
            fetch('http://localhost:5000/kitchen_confirm', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ confirmed: kitchenConfirmation }) // Send as "true"
            })
            .then(response => response.json())
            .then(data => console.log(data))
            .catch(error => console.error('Error:', error));

            alert(`Kitchen readiness confirmed for Table ${tableNumber}.`);
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


    // Customer: Confirm table readiness
    document.getElementById('confirm_table_customer_button').addEventListener('click', () => {
        const tableNumber = tableSelectCustomer.value;
        if (tableNumber) {
            const tableConfirmation = true; // Send "true" as a string for confirmation

            // Send table confirmation to backend as "true"
            fetch('http://localhost:5000/table_confirm', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ confirmed: tableConfirmation }) // Send as "true"
            })
            .then(response => response.json())
            .then(data => console.log(data))
            .catch(error => console.error('Error:', error));

            alert(`Table ${tableNumber} confirmed by customer.`);
        } else {
            alert("Please select a table to confirm.");
        }
    });
  
    // Helper function to update the task queue display
  // Update task queue display function
  function updateTaskQueue() {
      const taskQueueDiv = document.getElementById('task_queue_section');

      if (taskQueue.length === 0) {
          taskQueueDiv.innerHTML = "No tasks in the queue.";
      } else {
          // Remove all existing task items before adding new ones
          taskQueueDiv.innerHTML = ''; 

          taskQueue.forEach(task => {
              const taskItem = document.createElement('div');
              taskItem.textContent = `${task.table} - ordered`;
              taskQueueDiv.appendChild(taskItem);
          });
      }
  }


    // Helper function to update the table confirmation display
    function updateTableConfirmationDisplay() {
        const tableConfirmDiv = document.getElementById('table_confirmations');
        tableConfirmDiv.innerHTML = '';

        for (const table in tableConfirmations) {
            const tableDiv = document.createElement('div');
            tableDiv.textContent = `Table${table}: ${tableConfirmations[table] ? 'Confirmed' : 'Not Confirmed'}`;
            tableConfirmDiv.appendChild(tableDiv);
        }
    }

    // Helper function to update the kitchen confirmation display
    function updateKitchenConfirmationDisplay() {
        const kitchenConfirmDiv = document.getElementById('kitchen_confirmations');
        kitchenConfirmDiv.innerHTML = '';

        for (const table in kitchenConfirmations) {
            const kitchenDiv = document.createElement('div');
            kitchenDiv.textContent = `Table${table}: ${kitchenConfirmations[table] ? 'Ready' : 'Not Ready'}`;
            kitchenConfirmDiv.appendChild(kitchenDiv);
        }
    }

    // Initialize the page
    generateTableOptions();
});


